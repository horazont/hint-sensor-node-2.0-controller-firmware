#include "clock.h"

#include <stm32f10x.h>

#include "cpusample.h"

static volatile uint16_t m_now = 0;


static inline void _rtc_wait_for_write()
{
    while (!(RTC->CRL & RTC_CRL_RTOFF));
}


void SysTick_Handler()
{
    m_now += 1;
#ifdef SAMPLE_CPU
    static uint8_t sample_tick = 0;
    static uint16_t sample_tick_rand = 18213;
    // this switch makes us sample the current CPU task in intervals of 2-3-5-7,
    // which together is another prime interval of 17ms. this should give us
    // decent coverage of common task usage.
    //
    // overview of the timed things which happen, with their prime factors:
    // - ADC sample period (no CPU involvement): 0.05ms
    // - Accel sample period: 5ms (5)
    // - ADC buffer fill: 12.8ms (2, 3; 13)
    // - Accel buffer fill: 60ms (2, 3, 5)
    // - Light sensor channel sample interval: 200ms (2, 5)
    // - Compass sample period: 320ms (2, 5)
    // - Status update: 487ms (487)
    // - Light sensor sampling period: 1000ms (2, 5)
    // - Compass buffer fill: 6400ms (2, 5)
    // - BME280 sampling period: 2500ms (2, 5)
    // - OneWire sampling period: 10000ms (2, 5)
    //
    // so while 2, 3 and 5 may not be very useful, 7 (and the overall period of
    // 17) should give decent sampling
    /* switch (sample_tick)
    {
    case 16:
        sample_tick = 255;
        // fall through
    case 1:
    case 4:
    case 9:
        cpu_user::acquire_sample();
        // fall through
    default:
        sample_tick += 1;
    };*/
    if (sample_tick == 0) {
        cpu_user::acquire_sample();
        sample_tick_rand = sample_tick_rand * 5 + 3;
        sample_tick = (sample_tick_rand >> 12) + 2;
    } else {
        sample_tick -= 1;
    }
#endif
}

stm32_clock::time_point stm32_clock::now()
{
    return time_point(duration(m_now));
}

uint16_t stm32_clock::now_raw()
{
    return m_now;
}

void stm32_clock::init()
{
    SysTick_Config(72000);
    /*TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    TIM1->SMCR = 0;
    TIM1->DIER = 0;

    // TIM1 runs at CPU clock speed
    TIM1->PSC = 1000;
    TIM1->ARR = 36000;*/
}

void stm32_clock::enable()
{
}


uint32_t stm32_rtc::now_raw()
{
    return (RTC->CNTH << 16) | RTC->CNTL;
}

void stm32_rtc::init()
{
    PWR->CR |= PWR_CR_DBP;
    RCC->BDCR = (RCC->BDCR & ~(RCC_BDCR_RTCSEL))
            | RCC_BDCR_RTCSEL_HSE
            | RCC_BDCR_RTCEN;

    _rtc_wait_for_write();
    // enter configuration mode
    RTC->CRL |= RTC_CRL_CNF;
    RTC->PRLH = 0;
    // we are fast by 0.8%
    RTC->PRLL = 61;
    RTC->CRL &= ~RTC_CRL_CNF;
    _rtc_wait_for_write();
}

void stm32_rtc::enable()
{

}


void usleep(uint32_t us)
{
    const uint16_t ms = (us > 65535000 ? 65535 : us / 1000);
    const uint16_t start = stm32_clock::now_raw();
    const uint16_t end = start + ms;
    while (end <= m_now) {
        __WFI();
    }
    while (m_now < end) {
        __WFI();
    }
}

void usleep_interruptible(uint32_t us)
{
    if (us < 1000) {
        return;
    }
    __WFI();
}
