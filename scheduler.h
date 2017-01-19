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

        inline bool can_run_now(const sched_clock::time_point now) const
        {
            switch (condition.type) {
            case WakeupCondition::NONE:
                return true;
            case WakeupCondition::FINSIHED:
                return false;
            case WakeupCondition::TIMER:
                return condition.wakeup_at <= now;
            case WakeupCondition::EVENT:
                return *condition.event_bits == 0;
            }
            return false;
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
        m_timed_tasks(),
        m_timed_tasks_end(m_timed_tasks.begin()),
        m_event_tasks(),
        m_event_tasks_end(m_event_tasks.begin())
    {

    }

private:
    TaskArray m_tasks;
    TaskPArray m_timed_tasks;
    typename TaskPArray::iterator m_timed_tasks_end;
    TaskPArray m_event_tasks;
    typename TaskPArray::iterator m_event_tasks_end;

private:
    void add_timed_task(Task &task)
    {
        *m_timed_tasks_end++ = &task;
        std::push_heap(m_timed_tasks.begin(), m_timed_tasks_end, &timed_task_cmp);
    }

public:
    template <typename coroutine_t, typename... arg_ts>
    bool add_task(coroutine_t *coro, arg_ts&&... args)
    {
        for (Task &task: m_tasks) {
            if (!task.coro) {
                (*coro)(args...);
                task.coro = coro;
                task.condition.type = WakeupCondition::NONE;
                *m_event_tasks_end++ = &task;
                return true;
            }
        }
        return false;
    }

    void run()
    {
        while (1) {
            sched_clock::time_point now = sched_clock::now();

            while (m_timed_tasks_end != m_timed_tasks.begin()
                   && m_timed_tasks.front()->condition.wakeup_at <= now)
            {
                std::pop_heap(m_timed_tasks.begin(), m_timed_tasks_end);
                Task &task = **(m_timed_tasks_end-1);
                task.condition = task.coro->step(now);
                switch (task.condition.type)
                {
                case WakeupCondition::TIMER:
                {
                    // re-add the task to the queue, no need to modify
                    // m_timed_tasks_end
                    std::push_heap(m_timed_tasks.begin(), m_timed_tasks_end);
                    break;
                }
                case WakeupCondition::NONE:
                case WakeupCondition::EVENT:
                {
                    // move task to event list
                    *m_event_tasks_end++ = &task;
                    // intentionall fall-through
                }
                case WakeupCondition::FINSIHED:
                {
                    // remove task
                    m_timed_tasks_end--;
                    break;
                }
                }
            }

            for (auto iter = m_event_tasks.begin();
                 iter != m_event_tasks_end;
                 ++iter)
            {
                Task &task = **iter;
                if (!task.can_run_now(now)) {
                    continue;
                }

                task.condition = task.coro->step(now);
                switch (task.condition.type)
                {
                case WakeupCondition::TIMER:
                {
                    add_timed_task(task);
                    // intentional fallthrough
                }
                case WakeupCondition::FINSIHED:
                {
                    if (iter != m_event_tasks_end-1) {
                        std::swap(*iter, *(m_event_tasks_end-1));
                    }
                    iter--;
                    m_event_tasks_end--;
                    break;
                }
                case WakeupCondition::NONE:
                {
                    // we must not sleep, NONE wants to run again
                    // immediately
                    sched_no_event_pending.clear();
                    break;
                }
                case WakeupCondition::EVENT:;
                }
            }

            if (m_event_tasks_end == m_event_tasks.begin() &&
                    m_timed_tasks_end == m_timed_tasks.begin())
            {
                // no tasks to run anymore!
                return;
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
    sched_no_event_pending.clear();
}

#endif // SCHEDULER_H
