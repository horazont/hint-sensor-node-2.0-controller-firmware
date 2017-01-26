#ifndef SBX_LIGHTSENOSR_PERIOD_H
#define SBX_LIGHTSENSOR_PERIOD_H

#include <stm32f10x.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LS_PERIOD_SIGNAL_TIMER TIM3
#define LS_PERIOD_SIGNAL_RCC_ENR RCC_APB1ENR_TIM3EN
#define LS_PERIOD_PIN_CR GPIOA->CRL
#define LS_PERIOD_PIN_CONFIG(what, bit) GPIO_CRL_ ## what ## 6_ ## bit
#define LS_PERIOD_PIN_CONFIG_MASK (               \
        LS_PERIOD_PIN_CONFIG(CNF, 0) |            \
        LS_PERIOD_PIN_CONFIG(CNF, 1) |            \
        LS_PERIOD_PIN_CONFIG(MODE, 0) |           \
        LS_PERIOD_PIN_CONFIG(MODE, 1)             \
        )

void ls_period_init();
void ls_period_enable();
void ls_period_disable();

static inline uint16_t ls_period_read_full()
{
    return LS_PERIOD_SIGNAL_TIMER->CCR1;
}

static inline uint16_t ls_period_read_half()
{
    return LS_PERIOD_SIGNAL_TIMER->CCR2;
}

#ifdef __cplusplus
}
#endif

#endif
