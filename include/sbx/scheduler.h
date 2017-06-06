#ifndef SBX_SCHEDULER_H
#define SBX_SCHEDULER_H

#include <algorithm>
#include <atomic>
#include <array>
#include <cstdint>

#include <stm32f10x.h>

#include "coroutine.h"

extern std::atomic_flag sched_no_event_pending;

template <std::size_t max_tasks>
class Scheduler
{
private:
    struct Task
    {
        Task():
            coro(nullptr)
        {

        }

        WakeupCondition condition;
        Coroutine *coro;

        inline bool is_dead() const
        {
            return !coro || condition.type == WakeupCondition::FINSIHED;
        }
    };

    static bool timed_task_cmp(const Task *a, const Task *b)
    {
        return a->condition.wakeup_at > b->condition.wakeup_at;
    }

    using TaskArray = std::array<Task, max_tasks>;
    using TaskPArray = std::array<Task*, max_tasks>;

public:
    Scheduler():
        m_tasks(),
        m_tasks_end(m_tasks.begin())
    {

    }

private:
    TaskArray m_tasks;
    typename TaskArray::iterator m_tasks_end;

public:
    template <typename coroutine_t, typename... arg_ts>
    bool add_task(coroutine_t *coro, arg_ts&&... args)
    {
        if (m_tasks_end == m_tasks.end()) {
            __asm__ volatile("bkpt #30"); // scheduler configured for less tasks
            return false;
        }
        Task &task = *m_tasks_end++;
        (*coro)(args...);
        task.coro = coro;
        task.condition.type = WakeupCondition::NONE;
        return true;
    }

    void run()
    {
        while (1) {
            sched_clock::time_point now = sched_clock::now();

            for (auto iter = m_tasks.begin();
                 iter != m_tasks_end;
                 ++iter)
            {
                Task &task = *iter;
                if (!task.condition.can_run_now(now)) {
                    continue;
                }
                sched_clock::time_point wakeup_timestamp = now;
                if (task.condition.type == WakeupCondition::TIMER) {
                    wakeup_timestamp = task.condition.wakeup_at;
                }
                task.condition = task.coro->step(wakeup_timestamp);
            }

            if (sched_no_event_pending.test_and_set()) {
                // wait for next interrupt
                __WFI();
            }
        }
    }

};

inline void set_pending_event()
{
    sched_no_event_pending.clear(std::memory_order_relaxed);
}

#endif // SCHEDULER_H
