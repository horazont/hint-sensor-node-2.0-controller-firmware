#ifndef CLOCK_H
#define CLOCK_H

#include <chrono>

template <typename clock_t, typename duration_t>
struct wraparound_timepoint_t
{
    typedef clock_t clock;
    typedef duration_t duration;
    typedef typename duration::rep rep;
    typedef typename duration::period period;

    wraparound_timepoint_t() = delete;

    wraparound_timepoint_t &operator+=(const duration &dur)
    {
        m_d += dur;
        return *this;
    }

    wraparound_timepoint_t &operator-=(const duration &dur)
    {
        m_d -= dur;
        return *this;
    }

private:
    wraparound_timepoint_t(const duration &dur);

    duration m_d;

    friend struct stm32_clock;
};


struct stm32_clock
{
  typedef std::chrono::duration<uint16_t, std::milli> duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef std::chrono::time_point<stm32_clock, duration> 	time_point;

  static constexpr bool is_steady = true;

  static time_point now() noexcept;
  static uint16_t now_raw() noexcept;

  static void init() noexcept;
  static void enable() noexcept;
};

void usleep(uint32_t us);

/**
 * Sleep for at most the given amount of microseconds.
 *
 * If an interrupt occurs before the period of time is elapsed, the function
 * returns early.
 *
 * @param us Number of microseconds to sleep for.
 */
void usleep_interruptible(uint32_t us);

#ifdef __cplusplus
extern "C" {
#endif
void SysTick_Handler();
#ifdef __cplusplus
}
#endif

#endif // CLOCK_H
