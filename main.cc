#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

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
    while (1);
}

void HardFault_Handler()
{
    while (1);
}

void MemManage_Handler()
{
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
            await(csleep(500, now));
            GPIOA->BSRR = GPIO_BSRR_BS5;
            await(csleep(500, now));
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        COROUTINE_END;
    }
};

void delay() {
    for (uint32_t i = 0; i < 800000; ++i) {
        __asm__ volatile("nop");
    }
}


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

    Scheduler<1> scheduler;
    BlinkLED blink;

    /* I2C1->CCR = 0 */
    /*     | 180; */
    /* I2C1->TRISE = 36+1; */

    /* I2C1->CR1 |= I2C_CR1_PE; */

    /* I2C1->CR2 = 0 */
    /*     | I2C_CR2_ITBUFEN */
    /*     | I2C_CR2_ITEVTEN */
    /*     | I2C_CR2_ITERREN */
    /*     | 36  // frequency of APB1 domain, in MHz */
    /*     ; */

    /* // I2C Interrupts */
    /* NVIC_EnableIRQ(31); */
    /* NVIC_EnableIRQ(32); */

    /* ls_freq_init(); */
    /* ls_freq_enable(); */
//    i2c_init();
//    i2c_enable();

//    puts("Hi!\n");
//    delay();

//    i2c_workaround_reset();

//    bool set = true;
//    // uint8_t x = 30;
//    /* uint8_t reg8[8] = {0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xbe, 0xef}; */
//    // uint16_t reg16[6] = {0xdead, 0xbeef, 0xdead, 0xbeef, 0xdead, 0xbeef};
//    static const uint8_t config_20[] = {
//        // 0x20
//        0x67,
//        0x00,
//    };
//    static const uint8_t config_24[] = {
//        // 0x24
//        0xf4,
//        0x00,
//        0x00,
//    };

//    i2c_smbus_write(0x1d, 0x20, 2, &config_20[0]);
//    delay();
//    i2c_smbus_write(0x1d, 0x24, 3, &config_24[0]);
//    delay();

    stm32_clock::init();
    stm32_clock::enable();

    scheduler.add_task(&blink);

    scheduler.run();

    while (1) {
//        if (set) {
//            GPIOA->BSRR = GPIO_BSRR_BS5;
//        } else {
//            GPIOA->BSRR = GPIO_BSRR_BR5;
//        }
//        set = !set;
//        usleep(500000);

//        {
//            char buf[6];
//            buf[4] = '\n';
//            buf[5] = 0;
//            uint16_to_hex(stm32_clock::now_raw() / 1000, buf);
//            puts(buf);
//        }

//        if (x == 32) {
//            /* I2C1->CR1 |= I2C_CR1_START | I2C_CR1_ACK; */
//            /* i2c_smbus_read(0x1d, 0x1f, 8, &reg8[0]); */
//            i2c_smbus_read(0x1d, 0x28, 3*2, (uint8_t*)&reg16[0]);
//            x = 0;
//        } else if (x == 30) {
//            char buf[6];
//            buf[4] = ' ';
//            buf[5] = 0;
//            puts("recvd = ");
//            for (uint8_t i = 0; i < 6; ++i) {
//                uint16_to_hex(reg16[i], buf);
//                puts(buf);
//            }
//        } else if (x == 31) {
//            char buf[6];
//            buf[4] = '\n';
//            buf[5] = 0;

//            puts("I2C SR2 = ");
//            uint16_to_hex(I2C1->SR2, buf);
//            puts(buf);

//            puts("I2C CR1 = ");
//            uint16_to_hex(I2C1->CR1, buf);
//            puts(buf);

//            puts("I2C CR2 = ");
//            uint16_to_hex(I2C1->CR2, buf);
//            puts(buf);
//        }
//        x += 1;


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
