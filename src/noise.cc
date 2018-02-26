#include "noise.h"

#include <stm32f10x.h>

#include "notify.h"
#include "usart.h"


static std::array<noise_buffer_t, 2> buffers;
static std::array<notifier_t, 2> notifiers;
static volatile uint8_t current_buffer;
static uint16_t sample_seq;

void noise_init()
{
    current_buffer = 0;
    sample_seq = 0;
    buffers[current_buffer].first_sample = sample_seq;

    const uint16_t period = 24;

    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    // this sets it up s.t. the ADC fires with 32kHz sample rate
    TIM1->PSC = 89;
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
    ADC1->SMPR2 = 0
            // sample ch 3 for 55.5 cycles
            | ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_2
            // sample ch 4 for 55.5 cycles
            | ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_2
            // sample ch 5 for 55.5 cycles
            | ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_2
            ;

    ADC1->CR2 = ADC_CR2_ADON;

    // give it a moment to settle
    for (uint32_t i = 0; i < 10000; ++i) {
        __asm__ volatile("nop");
    }

    ADC1->CR2 |= ADC_CR2_CAL;

    // wait for calibration to finish
    while (ADC1->CR2 & ADC_CR2_CAL);

    // prepare DMA
    DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
    DMA1_Channel1->CCR = 0
            // Medium Priority
            | DMA_CCR1_PL_0
            // Memory size 16 bits
            | DMA_CCR1_MSIZE_0
            // Peripherial size 16 bits
            | DMA_CCR1_PSIZE_0
            // Memory increment mode
            | DMA_CCR1_MINC
            // Read from peripherial
            | 0
            // Enable Transfer complete interrupt
            | DMA_CCR1_TCIE
            ;
    DMA1_Channel1->CNDTR = NOISE_BUFFER_LENGTH;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;

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
            // enable DMA
            | ADC_CR2_DMA
            // start ADC
            | ADC_CR2_ADON
            ;
}

static inline void setup_dma_buffers()
{
    DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
    DMA1_Channel1->CNDTR = NOISE_BUFFER_LENGTH;
    DMA1_Channel1->CMAR = (uint32_t)&buffers[current_buffer].samples[0];
    DMA1_Channel1->CCR |= DMA_CCR1_EN;
}

void noise_enable()
{
    setup_dma_buffers();

    TIM1->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

ASYNC_CALLABLE noise_full_buffer(const noise_buffer_t *&full_buffer)
{
    const uint_fast8_t curr_buffer = current_buffer;
    full_buffer = &buffers[curr_buffer];
    return notifiers[curr_buffer].ready_c();
}


static inline void swap_buffers()
{
    const uint_fast8_t curr_buffer = current_buffer;
    const uint_fast8_t next_buffer = curr_buffer ^ 1;
    notifiers[curr_buffer].trigger();
    notifiers[next_buffer].reset();
    buffers[next_buffer].first_sample = sample_seq;
    current_buffer = next_buffer;
    sample_seq += 1;
}


void DMA1_Channel1_IRQHandler()
{
    cpu_user intr(CPU_INTR_ADC_DMA);
    const uint32_t isr = DMA1->ISR;
    if (isr & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        swap_buffers();
        setup_dma_buffers();
    } else {
        // ?? clear all the interrupts
        DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 |
                DMA_IFCR_CTEIF1;
    }

}
