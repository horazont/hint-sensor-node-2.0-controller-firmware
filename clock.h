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

    wraparound_timepoint_t():
        m_d(0)
    {

    }

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

    inline wraparound_timepoint_t operator+(const duration &dur) const
    {
        wraparound_timepoint_t result = *this;
        result += dur;
        return result;
    }

    bool operator<(const wraparound_timepoint_t other) const
    {
        const uint16_t t_this = m_d.count();
        const uint16_t t_other = other.m_d.count();
        const uint16_t other_based_on_this = t_other - t_this;
        return other_based_on_this < (1<<15);
    }

    inline bool operator>(const wraparound_timepoint_t other) const
    {
        return !(*this <= other);
    }

    inline bool operator<=(const wraparound_timepoint_t other) const
    {
        if (other == *this) {
            return true;
        }
        return *this < other;
    }

    inline bool operator==(const wraparound_timepoint_t other) const
    {
        return m_d == other.m_d;
    }

private:
    explicit wraparound_timepoint_t(const duration &dur):
        m_d(dur)
    {

    }

    duration m_d;

    friend struct stm32_clock;
};


struct stm32_clock
{
  typedef std::chrono::duration<uint16_t, std::milli> duration;
  typedef duration::rep rep;
  typedef duration::period period;
  typedef wraparound_timepoint_t<stm32_clock, duration> time_point;

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
