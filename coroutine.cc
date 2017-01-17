#include "coroutine.h"


Coroutine::Coroutine():
    m_state_line(0)
{

}

Coroutine::~Coroutine()
{

}

WakeupCondition Coroutine::step(const stm32_clock::time_point now)
{
    (void)now;
    COROUTINE_INIT;
    COROUTINE_END;
}





WakeupCondition sleepc(uint16_t ms)
{
    // we can at most sleep for 2^15-1
    if (ms >= 1<<15) {
        ms = (1<<15)-1;
    }
    return WakeupCondition{
        .type = WakeupCondition::TIMER,
        .wakeup_at = sched_clock::now() + std::chrono::milliseconds(ms),
        .event_bits = nullptr,
    };
}

WakeupCondition sleepc(uint16_t ms, const sched_clock::time_point now)
{
    // we can at most sleep for 2^15-1
    if (ms >= 1<<15) {
        ms = (1<<15)-1;
    }
    return WakeupCondition{
        .type = WakeupCondition::TIMER,
        .wakeup_at = now + std::chrono::milliseconds(ms),
        .event_bits = nullptr,
    };
}
