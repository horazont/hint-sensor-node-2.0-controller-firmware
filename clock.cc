#include "clock.h"

#include <stm32f10x.h>

static volatile uint16_t m_now = 0;


void SysTick_Handler()
{
    m_now += 1;
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
}

void stm32_clock::enable()
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
