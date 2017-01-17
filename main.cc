#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <cstring>

#include <stm32f10x.h>

#include "i2clib.h"
#include "clock.h"
#include "utils.h"
#include "scheduler.h"

#define STACK_TOP (void*)(0x20002000)

int main();

extern "C" {

void NMI_Handler()
{
    __ASM volatile("bkpt #03");
    while (1);
}

void HardFault_Handler()
{
    __ASM volatile("bkpt #04");
    while (1);
}

void MemManage_Handler()
{
    __ASM volatile("bkpt #05");
    while (1);
}

}


class BlinkLED: public Coroutine
{
public:
    void operator()()
    {

    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            await(sleepc(500, now));
            GPIOA->BSRR = GPIO_BSRR_BS5;
            await(sleepc(500, now));
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        COROUTINE_END;
    }
};


class I2CSensor: public Coroutine
{
private:
    uint8_t m_notify;
    uint8_t m_reg8[8];
    uint16_t m_reg16[6];
    char m_buf[6];

public:
    void operator()()
    {
        memset(&m_reg8[0], 0x42, sizeof(m_reg8));
        m_reg16[0] = 0xdead;
        m_reg16[1] = 0xbeef;
        m_reg16[2] = 0xdead;
        m_reg16[3] = 0xbeef;
        m_reg16[4] = 0xdead;
        m_reg16[5] = 0xbeef;
        m_buf[2] = ' ';
        m_buf[3] = 0;
        m_buf[4] = ' ';
        m_buf[5] = 0;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        static const uint8_t config_20[] = {
            // 0x20
            0x67,
            0x00,
        };
        static const uint8_t config_24[] = {
            // 0x24
            0xf4,
            0x00,
            0x00,
        };

        await(i2c_smbus_writec(0x1d, 0x20, 2, &config_20[0]));
        await(i2c_smbus_writec(0x1d, 0x24, 3, &config_24[0]));
        await(i2c_smbus_readc(0x1d, 0x1f, 8, &m_reg8[0]));
        puts("recvd = ");
        for (uint8_t i = 0; i < 8; ++i) {
            uint8_to_hex(m_reg8[i], m_buf);
            puts(m_buf);
        }
        puts("\n");

//        uint16_to_hex(DMA1_Channel7->CNDTR, m_buf);
//        puts(m_buf);
//        puts("\n");
//        m_buf[2] = ' ';
//        m_buf[3] = 0;

        while (1)
        {
            await(sleepc(1000, now));
            await(i2c_smbus_readc(0x1d, 0x28, 3*2, (uint8_t*)&m_reg16[0]));
            await(i2c_smbus_readc(0x1d, 0x08, 3*2, (uint8_t*)&m_reg16[3]));
            puts("recvd = ");
            for (uint8_t i = 0; i < 6; ++i) {
                uint16_to_hex(m_reg16[i], m_buf);
                puts(m_buf);
            }
        }
        COROUTINE_END;
    }
};

void delay() {
    for (uint32_t i = 0; i < 800000; ++i) {
        __asm__ volatile("nop");
    }
}


static BlinkLED blink;
static I2CSensor sensor;
static Scheduler<2> scheduler;


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
    RCC->AHBENR |= 0
            | RCC_AHBENR_DMA1EN;

    USART2->BRR = 312;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    I2C1->CR1 = 0;

    i2c_init();
    i2c_enable();
    i2c_workaround_reset();

    stm32_clock::init();
    stm32_clock::enable();

    scheduler.add_task(&blink);
    scheduler.add_task(&sensor);

    scheduler.run();

    while (1) {}

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
