#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f10x.h>

#include "lightsensor_freq.h"

#define STACK_TOP (void*)(0x20002000)

int main();

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


void delay() {
    for (uint32_t i = 0; i < 800000; ++i) {
        __asm__ volatile("nop");
    }
}

static const char hello_world[] = "Hello World!";


int main() {
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM4RST;
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_IOPCRST | RCC_APB2RSTR_IOPDRST | RCC_APB2RSTR_IOPERST;

    delay();

    RCC->APB1RSTR = 0;
    RCC->APB2RSTR = 0;

    GPIOA->CRL =
        GPIO_CRL_CNF0_0
        | GPIO_CRL_CNF1_0
        | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1
        | GPIO_CRL_CNF3_0
        | GPIO_CRL_CNF4_0
        | GPIO_CRL_MODE5_1
        | GPIO_CRL_CNF6_0
        | GPIO_CRL_CNF7_0;
    GPIOA->BSRR = GPIO_BSRR_BR5;

    GPIOD->CRL =
        GPIO_CRL_CNF0_0
        | GPIO_CRL_CNF1_0
        | GPIO_CRL_CNF2_0
        | GPIO_CRL_CNF3_0
        | GPIO_CRL_CNF4_0
        | GPIO_CRL_CNF5_0
        | GPIO_CRL_CNF6_0
        | GPIO_CRL_CNF7_0;

    USART2->BRR = 312;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    char buf[6];
    buf[5] = 0;
    buf[4] = '\n';

    ls_freq_init();
    ls_freq_enable();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;

    bool set = true;
    while (1) {
        if (set) {
            GPIOA->BSRR = GPIO_BSRR_BS5;
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        set = !set;
        delay();

        uint16_to_hex(GPIOD->IDR, buf);
        puts("GPIOD in: ");
        puts(buf);

        uint16_to_hex(TIM2->CNT, buf);
        puts("TIM2 CNT: ");
        puts(buf);

        uint16_to_hex(ls_freq_read(), buf);
        puts("TIM3 CCR1: ");
        puts(buf);

        uint16_to_hex(TIM3->CNT, buf);
        puts("TIM3 CNT: ");
        puts(buf);

        /* uint32_to_hex(TIM3->SR, buf); */
        /* puts("TIM3 SR: "); */
        /* puts(buf); */

        /* const uint16_t ccr1 = TIM3->CCR1; */
        /* const uint16_t ccr2 = TIM3->CCR2; */
        /* uint32_to_hex(ccr1, buf); */
        /* puts("TIM3 CCR1: "); */
        /* puts(buf); */

        /* uint32_to_hex(ccr2, buf); */
        /* puts("TIM3 CCR2: "); */
        /* puts(buf); */

        /* uint32_to_hex(TIM2->CNT, buf); */
        /* puts("TIM2 CNT: "); */
        /* puts(buf); */

        /* uint32_to_hex(TIM3->CNT, buf); */
        /* puts("TIM3 CNT: "); */
        /* puts(buf); */
        /* USART2->DR = 'x'; */
    }

    return 0;
}


#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
    USART2->DR = ch;
    while (!(USART2->SR & USART_FLAG_TC)) {

    }
    return ch;
}

int puts(const char *s) {
    while (*s != 0) {
        __io_putchar(*s);
        s++;
    }
    return 0;
}
