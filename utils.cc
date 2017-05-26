#include "utils.h"

char nybble_to_hex(const uint8_t nybble)
{
    const uint8_t safe_nybble = nybble & 0xF;
    if (safe_nybble >= 0xa) {
        return 87 + safe_nybble;
    } else {
        return 48 + safe_nybble;
    }
}

void uint32_to_hex(const uint32_t value, char *buf)
{
    uint32_t shift = 28;
    for (uint_fast8_t i = 0; i < 8; ++i) {
        const uint8_t nybble = (value >> shift) & 0xF;
        buf[i] = nybble_to_hex(nybble);
        shift -= 4;
    }
}

void uint16_to_hex(const uint16_t value, char *buf)
{
    uint16_t shift = 12;
    for (uint_fast8_t i = 0; i < 4; ++i) {
        const uint8_t nybble = (value >> shift) & 0xF;
        buf[i] = nybble_to_hex(nybble);
        shift -= 4;
    }
}

void uint8_to_hex(const uint8_t value, char *buf)
{
    uint8_t shift = 4;
    for (uint_fast8_t i = 0; i < 2; ++i) {
        const uint8_t nybble = (value >> shift) & 0xF;
        buf[i] = nybble_to_hex(nybble);
        shift -= 4;
    }
}

IRQn_Type get_dma_irqn(DMA_Channel_TypeDef *dmach)
{
    if (dmach == DMA1_Channel1) {
        return DMA1_Channel1_IRQn;
    } else if (dmach == DMA1_Channel2) {
        return DMA1_Channel2_IRQn;
    } else if (dmach == DMA1_Channel3) {
        return DMA1_Channel3_IRQn;
    } else if (dmach == DMA1_Channel4) {
        return DMA1_Channel4_IRQn;
    } else if (dmach == DMA1_Channel5) {
        return DMA1_Channel5_IRQn;
    } else if (dmach == DMA1_Channel6) {
        return DMA1_Channel6_IRQn;
    } else if (dmach == DMA1_Channel7) {
        return DMA1_Channel7_IRQn;
    } else {
        return NonMaskableInt_IRQn;
    }
}
