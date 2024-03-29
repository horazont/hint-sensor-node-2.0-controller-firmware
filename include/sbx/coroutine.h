#ifndef SBX_COROUTINE_H
#define SBX_COROUTINE_H

#include <array>
#include <chrono>
#include <cstdint>

#include "clock.h"

typedef stm32_clock sched_clock;

#define COROUTINE_INIT         (void)now; switch(m_state_line) { case 0:
#define COROUTINE_RETURN       return WakeupCondition::finished()
#define COROUTINE_END          } COROUTINE_RETURN;



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
            m_state_line=__LINE__ << 1;\
            static_assert(std::is_same<decltype(z), COROUTINE_RETURN_TYPE>::value, "non-awaitable passed to await.");\
            return (z); case __LINE__ << 1:;\
        } while (0)

#define yield await(WakeupCondition::none())

#define await_call(c, ...)\
        static_assert(!std::is_same<decltype(c), COROUTINE_RETURN_TYPE>::value, "awaitable passed to await_call; did you mean to use await or expand the argmuents?");\
        static_assert(std::is_base_of<Coroutine, std::remove_reference<decltype(c)>::type>::value, "first argument to await_call must be coroutine");\
        do {\
            c(__VA_ARGS__);\
            m_state_line=__LINE__ << 1;\
            case __LINE__ << 1:;\
            {\
                auto condition = c.step(now);\
                if (condition.type != WakeupCondition::FINSIHED) {\
                    return condition;\
                }\
            }\
            m_state_line=(__LINE__ << 1) | 1;\
            case (__LINE__ << 1) | 1:;\
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
    const volatile uint8_t *event_bits;

    inline static WakeupCondition finished()
    {
        WakeupCondition result;
        result.type = WakeupCondition::FINSIHED;
        result.event_bits = nullptr;
        return result;
    }

    inline static WakeupCondition event(const volatile uint8_t *event_bits)
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

    inline bool can_run_now(const sched_clock::time_point now) const
    {
        switch (type) {
        case WakeupCondition::NONE:
            return true;
        case WakeupCondition::FINSIHED:
            return false;
        case WakeupCondition::TIMER:
            return wakeup_at <= now;
        case WakeupCondition::EVENT:
            return *event_bits == 0;
        }
        return false;
    }
};

class Coroutine
{
public:
    Coroutine();
    Coroutine(const Coroutine &ref) = delete;
    Coroutine(Coroutine &&src) = delete;
    virtual ~Coroutine();

protected:
    uint32_t m_state_line;

protected:
    void operator()();

public:
    virtual WakeupCondition step(const sched_clock::time_point now);
};


WakeupCondition sleep_c(uint16_t ms);
WakeupCondition sleep_c(uint16_t ms, sched_clock::time_point now);


class WaitFor: public Coroutine
{
private:
    WakeupCondition m_waitee;
    WakeupCondition m_timeout;
    bool *m_timed_out;

public:
    void operator()(WakeupCondition waitee,
                    uint16_t timeout,
                    bool &timed_out)
    {
        m_waitee = waitee;
        m_timeout = sleep_c(timeout);
        m_timed_out = &timed_out;
        *m_timed_out = false;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (true) {
            if (m_waitee.can_run_now(now)) {
                *m_timed_out = false;
                COROUTINE_RETURN;
            }
            if (m_timeout.can_run_now(now)) {
                *m_timed_out = true;
                COROUTINE_RETURN;
            }
            yield;
        }
        COROUTINE_END;
    }

};

#endif
