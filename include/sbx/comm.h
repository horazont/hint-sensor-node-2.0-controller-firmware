#ifndef COMM_H
#define COMM_H

#include <array>
#include <limits>

#include "coroutine.h"
#include "notify.h"


enum BufferFlag: uint8_t
{
    PRIO_NO_RETRIES = 0x01,
    PRIO_EXTRA_RETRIES = 0x02,
};


template <std::size_t sram_to_use, std::size_t bytes_per_packet>
class CommBuffer
{
private:
    struct buffer_t
    {
        buffer_t():
            m_ready(false),
            m_length(0),
            m_flags(0)
        {

        }

        bool m_ready;
        uint16_t m_length;
        uint8_t m_flags;
        std::array<uint8_t, bytes_per_packet> m_data;
    };

public:
    static constexpr std::size_t BUFFER_COUNT = sram_to_use / sizeof(buffer_t);
    static constexpr std::size_t BUFFER_SIZE = bytes_per_packet;
    using buffer_handle_t = uint16_t;
    static constexpr buffer_handle_t INVALID_BUFFER = std::numeric_limits<buffer_handle_t>::max();

public:
    CommBuffer():
        m_buffers_allocated(0),
        m_buffers_ready(0),
        m_most_allocated(0),
        m_dequeue_ptr(0),
        m_allocate_ptr(0)
    {
        m_any_buffer_free_notifier.trigger();
    }

private:
    uint16_t m_buffers_allocated;
    uint16_t m_buffers_ready;
    uint16_t m_most_allocated;
    buffer_handle_t m_dequeue_ptr;
    buffer_handle_t m_allocate_ptr;
    std::array<buffer_t, BUFFER_COUNT> m_buffers;
    notifier_t m_any_buffer_ready_notifier;
    notifier_t m_any_buffer_free_notifier;

    inline void dequeue_buffer(buffer_handle_t i,
                               uint8_t *&buf, uint16_t &length,
                               uint8_t &flags)
    {
        buffer_t &buffer = m_buffers[i];
        m_dequeue_ptr = i+1;
        buffer.m_ready = false;
        length = buffer.m_length;
        flags = buffer.m_flags;
        buf = &buffer.m_data[0];
        m_buffers_ready--;
        if (m_buffers_ready == 0) {
            m_any_buffer_ready_notifier.reset();
        }
    }

    inline void allocate_buffer(buffer_handle_t i,
                                void *&buf,
                                const uint16_t length,
                                const uint8_t flags)
    {
        buffer_t &buffer = m_buffers[i];
        buffer.m_length = length;
        buffer.m_ready = false;
        buffer.m_flags = flags;
        buf = &buffer.m_data[0];
        m_buffers_allocated += 1;
        if (m_buffers_allocated == BUFFER_COUNT) {
            m_any_buffer_free_notifier.reset();
        }
        m_most_allocated = std::max(m_most_allocated, m_buffers_allocated);
        m_allocate_ptr = i+1;
    }

public:
    buffer_handle_t allocate(void *&buf, const uint16_t length = bytes_per_packet,
                             const uint8_t flags = 0)
    {
        if (m_buffers_allocated == BUFFER_COUNT) {
            return INVALID_BUFFER;
        }
        const buffer_handle_t end = m_allocate_ptr;
        buffer_handle_t i = m_allocate_ptr;
        for (; i < m_buffers.size(); ++i) {
            buffer_t &buffer = m_buffers[i];
            if (buffer.m_length > 0) {
                continue;
            }
            allocate_buffer(i, buf, length, flags);
            return i;
        }
        for (i = 0; i < end; ++i) {
            buffer_t &buffer = m_buffers[i];
            if (buffer.m_length > 0) {
                continue;
            }
            allocate_buffer(i, buf, length, flags);
            return i;
        }
        __asm__ volatile("bkpt #20");  // no buffer available, but counter is off
        return INVALID_BUFFER;
    }

    void set_ready(const buffer_handle_t buffer_handle,
                   uint16_t update_length = 0)
    {
#ifndef NDEBUG
        if (buffer_handle >= m_buffers.size() ||
                m_buffers[buffer_handle].m_length == 0 ||
                m_buffers[buffer_handle].m_ready)
        {
            __asm__ volatile("bkpt #20");  // attempt to set buffer ready which is not allocated, already ready or uses an invalid handle
        }
#endif
        buffer_t &buffer = m_buffers[buffer_handle];
        if (!buffer.m_ready) {
            buffer.m_ready = true;
            if (update_length) {
                buffer.m_length = update_length;
            }
            m_buffers_ready += 1;
            m_any_buffer_ready_notifier.trigger();
        }
    }

    buffer_handle_t dequeue(uint8_t *&buf, uint16_t &length,
                            uint8_t &flags)
    {
        if (m_buffers_ready == 0) {
            return INVALID_BUFFER;
        }
        const buffer_handle_t end = m_dequeue_ptr;
        buffer_handle_t i = m_dequeue_ptr;
        for (; i < m_buffers.size(); ++i) {
            buffer_t &buffer = m_buffers[i];
            if (!buffer.m_ready) {
                continue;
            }
            dequeue_buffer(i, buf, length, flags);
            return i;
        }
        for (i = 0; i < end; ++i) {
            buffer_t &buffer = m_buffers[i];
            if (!buffer.m_ready) {
                continue;
            }
            dequeue_buffer(i, buf, length, flags);
            return i;
        }
        __asm__ volatile("bkpt #20");  // no buffer ready, but counter is off
        return INVALID_BUFFER;
    }

    void release(buffer_handle_t buffer_handle)
    {
#ifndef NDEBUG
        if (buffer_handle >= m_buffers.size() ||
                m_buffers[buffer_handle].m_length == 0 ||
                m_buffers[buffer_handle].m_ready)
        {
            __asm__ volatile("bkpt #20");  // attempt to release which is not allocated, still ready or uses an invalid handle
        }
#endif
        buffer_t &buffer = m_buffers[buffer_handle];
        if (buffer.m_length) {
            buffer.m_length = 0;
            m_buffers_allocated--;
            m_any_buffer_free_notifier.trigger();
        }
    }

    ASYNC_CALLABLE any_buffer_ready()
    {
        if (m_buffers_ready > 0) {
            return WakeupCondition::none();
        }
        return m_any_buffer_ready_notifier.ready_c();
    }

    ASYNC_CALLABLE any_buffer_free()
    {
        if (m_buffers_allocated < BUFFER_COUNT) {
            return WakeupCondition::none();
        }
        return m_any_buffer_free_notifier.ready_c();
    }

    inline void fetch_stats_and_reset(
            uint16_t &most_allocated,
            uint16_t &current_allocated,
            uint16_t &current_ready)
    {
        most_allocated = m_most_allocated;
        current_allocated = m_buffers_allocated;
        current_ready = m_buffers_ready;
        m_most_allocated = m_buffers_allocated;
    }

};

#endif // COMM_H
