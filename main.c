#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f10x.h>

#include "lightsensor_freq.h"
#include "stm32f10x_i2c.h"

#define STACK_TOP (void*)(0x20002000)

static volatile uint8_t recvd_byte = 0xff;
static volatile uint8_t bus_addr = 0x1d;
static volatile uint8_t sub_addr = 0x0f;

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

void uint8_to_hex(const uint8_t value, char *buf)
{
    uint8_t shift = 4;
    for (uint_fast8_t i = 0; i < 2; ++i) {
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

void I2C1_EV_IRQHandler()
{
    static uint8_t next_read = 0;
    const uint8_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_SB) {
        USART2->DR = 's';
        // start generated
        I2C1->DR = bus_addr << 1 | next_read;
        /* if (next_read) { */
        /*     I2C1->CR1 = (I2C1->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP; */
        /* } */
    } else if (sr1 & I2C_SR1_ADDR) {
        // address generated
        // find out whether we were about to send or to receive
        const volatile uint8_t sr2 = I2C1->SR2;
        if (sr2 & I2C_SR2_TRA) {
            USART2->DR = 't';
            I2C1->DR = sub_addr;
            I2C1->CR1 |= I2C_CR1_START;
            // transmitter mode
            next_read = 1;
       } else {
            USART2->DR = 'r';
            // receiver mode
            next_read = 0;
            // we only want one byte
            const uint16_t cr1 = I2C1->CR1;
            I2C1->CR1 = (cr1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
        }
    } else if (sr1 & I2C_SR1_BTF) {
        USART2->DR = 'f';
    } else if (sr1 & I2C_SR1_TXE) {
        USART2->DR = 'B';
    } else if (sr1 & I2C_SR1_RXNE) {
        USART2->DR = 'R';
        recvd_byte = I2C1->DR;
    } else {
        USART2->DR = '?';
    }
}

void I2C1_ER_IRQHandler()
{
    const uint16_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_TIMEOUT) {
        USART2->DR = 'T';
    } else if (sr1 & I2C_SR1_PECERR) {
        USART2->DR = 'P';
    } else if (sr1 & I2C_SR1_SMBALERT) {
        USART2->DR = 'S';
    } else {
        USART2->DR = nybble_to_hex(sr1 >> 8);
    }
    I2C1->SR1 = 0;
    I2C1->CR1 |= I2C_CR1_STOP;
}

static const char hello_world[] = "Hello World!";


int main() {
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_I2C1RST;
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

    GPIOB->CRL =
        GPIO_CRL_CNF0_0
        | GPIO_CRL_CNF1_0
        | GPIO_CRL_CNF2_0
        | GPIO_CRL_CNF3_0
        | GPIO_CRL_CNF4_0
        | GPIO_CRL_CNF5_0
        | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0
        | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0
        ;

    /* char buf[6]; */
    /* buf[5] = 0; */
    /* buf[4] = '\n'; */

    RCC->APB2ENR |= 0
        | RCC_APB2ENR_IOPAEN
        | RCC_APB2ENR_IOPBEN
        | RCC_APB2ENR_IOPCEN
        | RCC_APB2ENR_IOPDEN
        | RCC_APB2ENR_IOPEEN
        | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= 0
        | RCC_APB1ENR_TIM3EN
        | RCC_APB1ENR_USART2EN
        | RCC_APB1ENR_TIM2EN
        | RCC_APB1ENR_TIM4EN
        | RCC_APB1ENR_I2C1EN
        ;

    USART2->BRR = 312;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    I2C1->CR1 = 0;

    I2C1->CCR = 0
        | 180;
    I2C1->TRISE = 36+1;

    I2C1->CR1 |= I2C_CR1_PE;

    I2C1->CR2 = 0
        | I2C_CR2_ITBUFEN
        | I2C_CR2_ITEVTEN
        | I2C_CR2_ITERREN
        | 36  // frequency of APB1 domain, in MHz
        ;

    // I2C Interrupts
    NVIC_EnableIRQ(31);
    NVIC_EnableIRQ(32);

    /* ls_freq_init(); */
    /* ls_freq_enable(); */

    GPIOB->BSRR = GPIO_BSRR_BS6 | GPIO_BSRR_BS7;

    puts("Hi!\n");
    delay();

    bool set = true;
    uint8_t x = 32;
    while (1) {
        if (set) {
            GPIOA->BSRR = GPIO_BSRR_BS5;
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        set = !set;
        delay();

        if (x == 32) {
            I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK;
            x = 0;
        } else if (x == 15) {
            char buf[4];
            buf[2] = '\n';
            buf[3] = 0;
            uint8_to_hex(recvd_byte, buf);
            puts("recvd = ");
            puts(buf);
        }
        x += 1;

        /* uint16_to_hex(GPIOB->IDR, buf); */
        /* puts("GPIOB = "); */
        /* puts(buf); */

        /* USART2->DR = 'x'; */
    }

    return 0;
}


#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
    while (!(USART2->SR & USART_FLAG_TC));
    USART2->DR = ch;
    while (!(USART2->SR & USART_FLAG_TC));
    return ch;
}

int puts(const char *s) {
    while (*s != 0) {
        __io_putchar(*s);
        s++;
    }
    return 0;
}
