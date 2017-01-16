#include "coroutine.h"


Coroutine::Coroutine():
    m_state_line(0)
{

}

Coroutine::~Coroutine()
{

}

WakeupCondition Coroutine::step()
{
    COROUTINE_INIT;
    COROUTINE_END;
}

WakeupCondition WakeupCondition::finished()
{
    WakeupCondition result;
    result.type = WakeupCondition::FINSIHED;
    result.event_bits = nullptr;
    return result;
}

WakeupCondition WakeupCondition::event(volatile uint8_t *event_bits)
{
    WakeupCondition result;
    result.type = WakeupCondition::EVENT;
    result.event_bits = event_bits;
    return result;
}

WakeupCondition csleep(uint32_t ms)
{
    return WakeupCondition{
        .type = WakeupCondition::TIMER,
        .wakeup_at = sched_clock::now() + std::chrono::milliseconds(ms),
        .event_bits = nullptr,
    };
}
