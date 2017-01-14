#ifndef SBX_LIGHTSENSOR_FREQ_H
#define SBX_LIGHTSENSOR_FREQ_H

#include <stm32f10x.h>

#define LS_FREQ_MASTER_TIMER TIM2
#define LS_FREQ_SLAVE_TIMER TIM3
#define LS_FREQ_MASTER_RCC_ENR RCC_APB1ENR_TIM2EN
#define LS_FREQ_SLAVE_RCC_ENR RCC_APB1ENR_TIM3EN
#define LS_FREQ_SLAVE_TS (TIM_SMCR_TS_0)
#define LS_FREQ_PIN_CR GPIOD->CRL
#define LS_FREQ_PIN_CONFIG(what, bit) GPIO_CRL_ ## what ## 2_ ## bit
#define LS_FREQ_PIN_CONFIG_MASK (               \
        LS_FREQ_PIN_CONFIG(CNF, 0) |            \
        LS_FREQ_PIN_CONFIG(CNF, 1) |            \
        LS_FREQ_PIN_CONFIG(MODE, 0) |           \
        LS_FREQ_PIN_CONFIG(MODE, 1)             \
    )

void ls_freq_init();
void ls_freq_enable();
void ls_freq_disable();

static inline uint16_t ls_freq_read()
{
    return LS_FREQ_SLAVE_TIMER->CCR1;
}

#endif
