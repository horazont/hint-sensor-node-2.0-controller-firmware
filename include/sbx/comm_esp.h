#ifndef COMM_ESP_H
#define COMM_ESP_H

#include "comm.h"
#include "comm_sbx.h"
#include "coroutine.h"
#include "usart.h"

class CommESPTX: public Coroutine
{
public:
    using buffer_t = CommBuffer<4096, MAX_FRAME_PAYLOAD_SIZE>;

private:
    struct tx_info_t
    {
        buffer_t::buffer_handle_t handle;
        uint8_t *buffer;
        uint16_t len;
        uint8_t flags;
        std::array<uint8_t, 17> frame_header;
        uint8_t checksum;
        notifier_t acked;
        USART::sendv_array_t sendv;
    };

    static constexpr uint8_t FRAME_SEQNR_BASE = 0xf8;

public:
    CommESPTX() = delete;
    explicit CommESPTX(USART &usart);

private:
    USART &m_usart;
    buffer_t m_buffer;

    std::array<tx_info_t, 2> m_tx;
    uint8_t m_tx_curr_buf;
    uint8_t m_tx_prev_buf;

    uint8_t m_tx_checksum_base;

    WakeupCondition m_tx_done;

public:
    void operator()()
    {
        Coroutine::operator()();
        m_tx_done = WakeupCondition::none();
    }

    inline buffer_t &buffer()
    {
        return m_buffer;
    }

    COROUTINE_DECL;

};


class CommESPRX: public Coroutine
{
public:
    using buffer_t = CommBuffer<1024, 256>;

private:
    enum RXState: uint8_t
    {
        RX_IDLE = 0,
        RX_FRAME_LENGTH_MSB,
        RX_FRAME_LENGTH_LSB,
        RX_DATA
    };

public:
    CommESPRX() = delete;
    explicit CommESPRX(USART &usart);

private:
    static CommESPRX *m_esp;

private:
    USART &m_usart;
    buffer_t m_buffer;

    struct {
        RXState state;
        uint16_t length;
        uint8_t checksum;
        buffer_t::buffer_handle_t handle;
        bool spurious_rxneie;
    } m_interrupt_state;

    uint16_t m_overruns;
    uint16_t m_checksum_errors;
    uint16_t m_errors;
    uint16_t m_unknown_frames;
    uint16_t m_skipped_bytes;

    uint16_t m_tx_retries;
    uint16_t m_tx_lost;

    buffer_t::buffer_handle_t m_handle;
    uint8_t *m_buf;
    uint8_t m_flags;
    uint16_t m_length;
    uint8_t m_expected_checksum;

private:
    /**
     * @return true if the frame can be released.
     */
    bool handle_frame();
    static void rx_data_cb(const uint8_t ch, const uint16_t sr);
    static void rx_done_cb(bool success);
    void set_rx_failed();

public:
    void operator()()
    {
        Coroutine::operator()();
        m_handle = buffer_t::INVALID_BUFFER;
    }

    inline buffer_t &buffer()
    {
        return m_buffer;
    }

    inline uint16_t overruns() const
    {
        return m_overruns;
    }

    inline uint16_t errors() const
    {
        return m_errors;
    }

    inline uint16_t checksum_errors() const
    {
        return m_checksum_errors;
    }

    inline uint16_t unknown_frames() const
    {
        return m_unknown_frames;
    }

    inline uint16_t skipped_bytes() const
    {
        return m_skipped_bytes;
    }

    inline uint16_t tx_retries() const
    {
        return m_tx_retries;
    }

    inline uint16_t tx_lost() const
    {
        return m_tx_lost;
    }

    COROUTINE_DECL;

};


#endif // COMM_ESP_H
