#include "lightsensor_freq.h"

void ls_freq_init()
{
    TIM2->CR1 = 0;
    TIM2->CR2 = 0
        | TIM_CR2_MMS_1  // trigger on update
        ;
    TIM2->PSC = 90;

    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = 0
        | TIM_SMCR_ECE  // use external trigger as clock source
        | TIM_SMCR_MSM  // master-slave-mode
        | TIM_SMCR_TS_0  // use TIM2 as trigger
        | TIM_SMCR_SMS_2  // reset mode
        ;
    TIM3->CCMR1 = 0
        | TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1  // use internal trigger
        ;
    TIM3->CCER = 0
        | TIM_CCER_CC1E  // enable CC1
        ;

    GPIOD->CRL = (GPIOD->CRL & ~LS_FREQ_PIN_CONFIG_MASK)
        | LS_FREQ_PIN_CONFIG(CNF, 0);
}

void ls_freq_enable()
{
    LS_FREQ_MASTER_TIMER->CR1 |= TIM_CR1_CEN;
    LS_FREQ_SLAVE_TIMER->CR1 |= TIM_CR1_CEN;
}

void ls_freq_disable()
{
    LS_FREQ_MASTER_TIMER->CR1 &= ~TIM_CR1_CEN;
    LS_FREQ_SLAVE_TIMER->CR1 &= ~TIM_CR1_CEN;
}
