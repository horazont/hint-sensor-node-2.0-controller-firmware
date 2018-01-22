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
            coro(nullptr),
            cpu_ticks(0)
        {

        }

        WakeupCondition condition;
        Coroutine *coro;
        uint32_t cpu_ticks;

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
        m_tasks_end(m_tasks.begin()),
        m_idle_ticks(0)
    {

    }

private:
    TaskArray m_tasks;
    typename TaskArray::iterator m_tasks_end;

    sched_clock::time_point m_prev;
    sched_clock::time_point m_curr;
    uint16_t m_idle_ticks;

    uint16_t account_ticks() {
        m_prev = m_curr;
        m_curr = sched_clock::now();
        return (m_curr - m_prev).count();
    }

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

    inline uint16_t idle_ticks() const {
        return m_idle_ticks;
    }

    inline uint16_t task_ticks(const unsigned index) const {
        return m_tasks[index].cpu_ticks;
    }

    inline unsigned task_count() const {
        return m_tasks_end - m_tasks.begin();
    }

    void run()
    {
        m_prev = m_curr = sched_clock::now();
        while (1) {
            const sched_clock::time_point now = sched_clock::now();

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
                m_idle_ticks += account_ticks();
                task.condition = task.coro->step(wakeup_timestamp);
                task.cpu_ticks += account_ticks();
            }

            if (sched_no_event_pending.test_and_set()) {
                // wait for next interrupt
                m_idle_ticks += account_ticks();
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
