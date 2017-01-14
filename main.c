#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "stm32f10x.h"

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


void delay() {
    for (uint32_t i = 0; i < 800000; ++i) {
        __asm__ volatile("nop");
    }
}

static const char hello_world[] = "Hello World!";


int main() {
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST;
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

    USART2->BRR = 312;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    char buf[10];
    buf[9] = 0;
    buf[8] = '\n';

    TIM2->CR1 = 0;
    TIM2->CR2 = 0;
    TIM2->SMCR =
        TIM_SMCR_TS_1  // ITR2 (== TIM3)
        | TIM_SMCR_MSM
        | TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2  // External Clock Mode 1
        ;

    TIM3->CR1 = TIM_CR1_URS;
    TIM3->CR2 = TIM_CR2_MMS_1;
    TIM3->SR = 0;
    TIM3->PSC = 23;
    TIM3->DIER = 0;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;
    TIM3->CCMR2 = 0;
    TIM3->CCER = TIM_CCER_CC2P;
    TIM3->SMCR = 0
        | TIM_SMCR_MSM
        | TIM_SMCR_TS_0 | TIM_SMCR_TS_2
        | TIM_SMCR_SMS_2;
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    TIM3->CR1 |= TIM_CR1_CEN;
    TIM2->CR1 |= TIM_CR1_CEN;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN;

    bool set = true;
    while (1) {
        if (set) {
            GPIOA->BSRR = GPIO_BSRR_BS5;
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        set = !set;
        delay();

        uint32_to_hex(TIM3->CR1, buf);
        puts("TIM3 CR1: ");
        puts(buf);

        uint32_to_hex(TIM3->SR, buf);
        puts("TIM3 SR: ");
        puts(buf);

        /* const uint16_t ccr1 = TIM3->CCR1; */
        /* const uint16_t ccr2 = TIM3->CCR2; */
        /* uint32_to_hex(ccr1, buf); */
        /* puts("TIM3 CCR1: "); */
        /* puts(buf); */

        /* uint32_to_hex(ccr2, buf); */
        /* puts("TIM3 CCR2: "); */
        /* puts(buf); */

        uint32_to_hex(TIM2->CNT, buf);
        puts("TIM2 CNT: ");
        puts(buf);

        uint32_to_hex(TIM3->CNT, buf);
        puts("TIM3 CNT: ");
        puts(buf);
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
