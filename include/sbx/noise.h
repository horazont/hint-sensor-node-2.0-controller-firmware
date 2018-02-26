#ifndef SBX_NOISE_H
#define SBX_NOISE_H

#include <array>
#include <cstdint>

#include "coroutine.h"


static constexpr std::size_t NOISE_BUFFER_LENGTH = 256;

using noise_sample_array_t = std::array<volatile uint16_t, NOISE_BUFFER_LENGTH>;

struct noise_buffer_t
{
    volatile uint16_t first_sample;
    noise_sample_array_t samples;
};


void noise_init();
void noise_enable();

ASYNC_CALLABLE noise_full_buffer(
        const noise_buffer_t *&full_buffer);

void noise_get_state(uint16_t &last_seq,
                     uint16_t &last_timestamp);

#ifdef __cplusplus
extern "C" {
#endif

void ADC1_2_IRQHandler();
void DMA1_Channel1_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
