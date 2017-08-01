#include "lightsensor_freq.h"

void ls_freq_init()
{
    LS_FREQ_MASTER_TIMER->CR1 = 0;
    LS_FREQ_MASTER_TIMER->CR2 = 0
        | TIM_CR2_MMS_1  // trigger on update
        ;
    LS_FREQ_MASTER_TIMER->PSC = 90;

    LS_FREQ_SLAVE_TIMER->CR1 = 0;
    LS_FREQ_SLAVE_TIMER->CR2 = 0;
    LS_FREQ_SLAVE_TIMER->SMCR = 0
        | TIM_SMCR_ECE  // use external trigger as clock source
        | TIM_SMCR_MSM  // master-slave-mode
        | LS_FREQ_SLAVE_TS  // use TIM2 as trigger
        | TIM_SMCR_SMS_2  // reset mode
        ;
    LS_FREQ_SLAVE_TIMER->CCMR1 = 0
        | TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC1S_1  // use internal trigger
        ;
    LS_FREQ_SLAVE_TIMER->CCER = 0
        | TIM_CCER_CC1E  // enable CC1
        ;

    ls_freq_select_channel(0);
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

void ls_freq_select_channel(uint8_t ch)
{
    // clip
    ch = ch & 0x3;
    GPIOA->ODR = (GPIOA->ODR & ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1)) | (ch << 0);
}
