#ifndef SBX_NOISE_H
#define SBX_NOISE_H

#include <array>
#include <cstdint>

#include "coroutine.h"

static constexpr std::size_t ADC_BUFFER_LENGTH = 12;

struct adc_buffer_t
{
    uint16_t first_sample;
    std::array<uint16_t, ADC_BUFFER_LENGTH> samples;
};

void adc_init();
void adc_enable();

ASYNC_CALLABLE adc_sample(const uint8_t channel,
                          uint16_t &dest);

#ifdef __cplusplus
extern "C" {
#endif

void ADC1_2_IRQHandler();
void TIM1_CC_IRQHandler();
void DMA1_Channel1_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
