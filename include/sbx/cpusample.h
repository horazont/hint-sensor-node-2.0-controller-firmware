#ifndef CPUSAMPLE_H
#define CPUSAMPLE_H

#include <atomic>
#include <cstdint>

#include "comm_sbx.h"

#define SAMPLE_CPU
#ifdef SAMPLE_CPU

struct cpu_user
{
public:
    cpu_user() = delete;
    explicit cpu_user(const sbx_cpu_context_id ctx_id):
        m_old_context(
            m_curr_context.exchange(ctx_id, std::memory_order_seq_cst)
            )
    {

    }

    cpu_user(const cpu_user &ref) = delete;
    cpu_user &operator=(const cpu_user &ref) = delete;
    cpu_user(cpu_user &&src) = delete;
    cpu_user &operator=(cpu_user &&ref) = delete;
    ~cpu_user()
    {
        m_curr_context.store(m_old_context, std::memory_order_seq_cst);
    }

private:
    static std::atomic<sbx_cpu_context_id> m_curr_context;
    const sbx_cpu_context_id m_old_context;

public:
    static inline sbx_cpu_context_id curr_context() {
        return m_curr_context;
    }

public:
    static volatile std::uint16_t cpu_samples[CPU_TASK_MAX+1];
    static constexpr std::size_t cpu_samples_size = sizeof(cpu_samples);

    static inline void acquire_sample() {
        const uint8_t ctx = m_curr_context.load(std::memory_order_seq_cst);
        cpu_samples[ctx] += 1;
    }

};

#else

struct cpu_user
{
public:
    cpu_user() = delete;
    explicit cpu_user(const cpu_context_id ctx_id) {};
    cpu_user(const cpu_user &ref) = delete;
    cpu_user &operator=(const cpu_user &ref) = delete;
    cpu_user(cpu_user &&src) = delete;
    cpu_user &operator=(cpu_user &&ref) = delete;

public:
    static inline cpu_context_id curr_context() {
        return 0;
    };
};


#endif

#endif // CPUSAMPLE_H
