#include "adc.h"

#include <stm32f10x.h>

#include "notify.h"
#include "usart.h"


static uint16_t *adc_dest = nullptr;
static notifier_t adc_done_notify;


void adc_init()
{
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
}

void adc_enable()
{
    ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC1_2_IRQn);
    adc_done_notify.reset();
}

ASYNC_CALLABLE adc_sample(const uint8_t channel, uint16_t &dest)
{
    (void)channel;
    adc_dest = &dest;
    ADC1->CR2 |= ADC_CR2_ADON;
    return adc_done_notify.ready_c();
}

void ADC1_2_IRQHandler()
{
    const uint16_t sr = ADC1->SR;
    if (sr & ADC_SR_EOC) {
        *adc_dest = ADC1->DR;
        adc_done_notify.trigger();
    }
}

void DMA1_Channel1_IRQHandler()
{

}
