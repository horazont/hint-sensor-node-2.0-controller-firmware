#ifndef SBX_COROUTINE_H
#define SBX_COROUTINE_H

#include <array>
#include <chrono>
#include <cstdint>

#include "clock.h"

typedef stm32_clock sched_clock;

#define COROUTINE_INIT         switch(m_state_line) { case 0:
#define COROUTINE_END          } return WakeupCondition::finished();



#define COROUTINE_RETURN_TYPE WakeupCondition
#define COROUTINE_DECL COROUTINE_RETURN_TYPE step(const sched_clock::time_point now __attribute__((unused))) override
#define COROUTINE_DEF(cls) COROUTINE_RETURN_TYPE cls::step(const sched_clock::time_point now __attribute__((unused)))

#define ASYNC_CALLABLE COROUTINE_RETURN_TYPE

#define COROUTINE(name) \
    class name: public Coroutine \
    { \
    private: \
    }; \

#define await(z)     \
        do {\
            m_state_line=__LINE__;\
            static_assert(std::is_same<decltype(z), COROUTINE_RETURN_TYPE>::value, "non-awaitable passed to await.");\
            return (z); case __LINE__:;\
        } while (0)

#define yield await(WakeupCondition::none())

#define await_call(c, ...)\
        do {\
            c(__VA_ARGS__);\
            m_state_line=__LINE__;\
            case __LINE__:;\
            auto condition = c.step(now);\
            if (condition.type != WakeupCondition::FINSIHED) {\
                return condition;\
            }\
        } while (0)


struct WakeupCondition
{
    enum WakeupConditionType
    {
        FINSIHED = 0,
        NONE = 1,
        TIMER = 2,
        EVENT = 3
    };

    WakeupConditionType type;
    sched_clock::time_point wakeup_at;
    volatile uint8_t *event_bits;

    inline static WakeupCondition finished()
    {
        WakeupCondition result;
        result.type = WakeupCondition::FINSIHED;
        result.event_bits = nullptr;
        return result;
    }

    inline static WakeupCondition event(volatile uint8_t *event_bits)
    {
        WakeupCondition result;
        result.type = WakeupCondition::EVENT;
        result.event_bits = event_bits;
        return result;
    }

    inline static WakeupCondition none()
    {
        WakeupCondition result;
        result.type = WakeupCondition::NONE;
        return result;
    }
};

class Coroutine
{
public:
    Coroutine();
    virtual ~Coroutine();

protected:
    uint32_t m_state_line;

public:
    virtual WakeupCondition step(const sched_clock::time_point now);
};


WakeupCondition sleepc(uint16_t ms);
WakeupCondition sleepc(uint16_t ms, sched_clock::time_point now);

#endif
