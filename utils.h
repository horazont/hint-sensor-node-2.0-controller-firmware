#ifndef SBX_UTILS_H
#define SBX_UTILS_H

#include <stm32f10x.h>
#include <stdint.h>

char nybble_to_hex(const uint8_t nybble);
void uint32_to_hex(const uint32_t value, char *buf);
void uint16_to_hex(const uint16_t value, char *buf);
void uint8_to_hex(const uint8_t value, char *buf);
IRQn_Type get_dma_irqn(DMA_Channel_TypeDef *dmach);

#endif
