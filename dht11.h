#ifndef DHT11_H
#define DHT11_H

#include "coroutine.h"

#include <stm32f10x.h>

void dht11_init();


void dht11_sample(
        uint16_t &humidity,
        uint16_t &temperature,
        bool &valid);

ASYNC_CALLABLE dht11_sample_c(
        uint16_t &humidity,
        uint16_t &temperature,
        bool &valid);

void dht11_sample_a(
        uint16_t &humidity,
        uint16_t &temperature,
        bool &valid);

void dht11_sample_prep(
        uint16_t &humidity,
        uint16_t &temperature,
        bool &valid);

ASYNC_CALLABLE dht11_sample_fire_c();

static inline ASYNC_CALLABLE dht11_trigger_c()
{
    GPIOB->BSRR = GPIO_BSRR_BR5;
    TIM1->CR1 |= TIM_CR1_CEN;
    return sleep_c(5);
}

extern "C" {
void TIM1_CC_IRQHandler();
}

#endif // DHT11_H
