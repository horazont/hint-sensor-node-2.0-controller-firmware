#ifndef NOTIFY_H
#define NOTIFY_H

#include "scheduler.h"

struct notifier_t
{
public:
    notifier_t():
        m_field(1)
    {

    }

    explicit notifier_t(bool ready):
        m_field(ready ? 0 : 1)
    {

    }

private:
    volatile uint8_t m_field;

public:
    inline void trigger()
    {
        m_field = 0;
        set_pending_event();
    }

    inline void reset()
    {
        m_field = 1;
    }

    inline bool ready() const
    {
        return m_field == 0;
    }

    inline ASYNC_CALLABLE ready_c() const
    {
        if (ready()) {
            return WakeupCondition::none();
        }
        return WakeupCondition::event(&m_field);
    }

    inline void wait_for_ready() const
    {
        while (!ready());
    }

};

#endif // NOTIFY_H
