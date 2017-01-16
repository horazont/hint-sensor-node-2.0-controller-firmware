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
#define COROUTINE_DECL COROUTINE_RETURN_TYPE step(const sched_clock::time_point now) override
#define COROUTINE_DEF(cls) COROUTINE_RETURN_TYPE cls::step(const sched_clock::time_point now)

#define COROUTINE(name) \
    class name: public Coroutine \
    { \
    private: \
    }; \

#define await(z)     \
        do {\
            m_state_line=__LINE__;\
            return (z); case __LINE__:;\
        } while (0)

#define yield await(WakeupCondition{.type=WakeupCondition::NONE})

#define await_call(c, ...)\
        do {\
            c(__VA_ARGS__);\
            m_state_line=__LINE__;\
            case __LINE__:;\
            auto condition = c.step();\
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

    static WakeupCondition finished();
    static WakeupCondition event(volatile uint8_t *event_bits);
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


WakeupCondition csleep(uint16_t ms);
WakeupCondition csleep(uint16_t ms, sched_clock::time_point now);

#endif
