#include "adc.h"

#include <stm32f10x.h>

#include "notify.h"
#include "usart.h"


static uint16_t *adc_dest = nullptr;
static notifier_t adc_done_notify;

/*
 * Plan: sample ADC with 48kHz sample rate (if possible -- need to check)
 */


void adc_init()
{
    const uint16_t period = 199;

    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    TIM1->PSC = 35999;  // -> timer runs at 48kHz
    TIM1->ARR = period;

    TIM1->CR1 = 0;
    TIM1->CR2 = 0
            // send CCx events on update event .. not sure if this’ll work
            // | TIM_CR2_CCDS
            ;

    TIM1->CCR1 = period / 2;
    TIM1->CCMR1 = 0
            // set to some PWM mode
            | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2
            ;
    TIM1->CCER = TIM_CCER_CC1E;
    TIM1->BDTR = 0
            | TIM_BDTR_MOE
            ;
    TIM1->DIER = TIM_DIER_CC1IE;

    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_TSVREFE;
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = 0
            ;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0
            | ADC_SQR3_SQ1_2
            ;

    ADC1->CR2 = ADC_CR2_ADON;

    // give it a moment to settle
    for (uint32_t i = 0; i < 10000; ++i) {
        __asm__ volatile("nop");
    }

    ADC1->CR2 |= ADC_CR2_CAL;

    // wait for calibration to finish
    while (ADC1->CR2 & ADC_CR2_CAL);

    ADC1->CR1 = 0
            // enable End-of-Conversion Interrupt
            | ADC_CR1_EOCIE
            // enable scan mode
            | ADC_CR1_SCAN
            ;

    ADC1->CR2 = 0
            | ADC_CR2_TSVREFE
            // enable external trigger
            | ADC_CR2_EXTTRIG
            // use TIM1_CH1 as trigger
            | 0 // | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2
            // start ADC
            | ADC_CR2_ADON
            ;
}

void adc_enable()
{
    TIM1->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(ADC1_2_IRQn);
    adc_done_notify.reset();
}

ASYNC_CALLABLE adc_sample(const uint8_t channel, uint16_t &dest)
{
    (void)channel;
    adc_dest = &dest;
    return adc_done_notify.ready_c();
}

void ADC1_2_IRQHandler()
{
    const uint16_t sr = ADC1->SR;
    if (sr & ADC_SR_EOC) {
        // we must read DR unconditionally to clear EOC bit
        const uint16_t dr = ADC1->DR;
        if (adc_dest) {
            *adc_dest = dr;
            adc_done_notify.trigger();
        }
    }
}

void DMA1_Channel1_IRQHandler()
{

}
