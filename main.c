#include <stdlib.h>
#include <stdint.h>

#include "stm32f10x.h"

#define STACK_TOP (void*)(0x20002000)

void nmi_handler();
void hardfault_handler();
int main();


void *myvectors[] __attribute__((section("vectors"))) = {
    STACK_TOP,
    main,
    nmi_handler,
    hardfault_handler,
    hardfault_handler,
    hardfault_handler,
    hardfault_handler,
};


void nmi_handler() {
    GPIOC->BSRR = 1<<8;
    return;
}

void hardfault_handler() {
    GPIOC->BSRR = 1<<9;
    return;
}


void delay() {
    uint32_t i = 10000;
    while (i-- > 0) {
        __asm__ volatile("nop");
    }
}

static const char hello_world[] = "Hello World!";


int main() {
    RCC->AHBENR |=
        (1 << 1)  // enable DMA1
        | (1 << 0)  // enable DMA2
        ;
    RCC->APB2ENR |=
        (1 << 14) // enable USART1
        | (1 << 9)  // enable ADC1
        | (1 << 5) // enable IO port D
        | (1 << 4) // enable IO port C
        | (1 << 2) // enable IO port A
        | (1 << 0) // enable alternate function I/O
        ;
    /* RCC->APB2RSTR |= (1 << 14); */
    GPIOC->CRH = 0x11; // enable two IOs for LEDs
    GPIOC->CRL = 0;
    GPIOA->CRL = 0x04;

    ADC1->CR2 =
        (1 << 0) // wake up ADC
        ;

    GPIOA->CRH =
        (0b11 << 4) | (0b10 << 6) // setup USART1 TX output
        ;
    GPIOC->BSRR = 1<<24;
    GPIOC->BSRR = 1<<25;

    USART1->BRR = 0x340;  // baud rate divider to 9600 baud at 8 MHz
    USART1->CR1 = (1 << 13) | (1 << 3); // enable USART, enable TX
    USART1->CR2 = 0;
    USART1->CR3 = (1 << 7); // enable DMA

    ADC1->CR1 = (1 << 8); // scan mode
    ADC1->CR2 =
        (1 << 8) // enable DMA
        | (1 << 1) // continuous mode
        ;
    ADC1->SQR1 = (12 << 0) | (13 << 5) | (16 << 10) | (17 << 15) | (0b1111 << 20);
    ADC1->SQR2 = (6 << 0) | (7 << 5) | (8 << 10) | (9 << 15) | (10 << 20) | (11 << 25);
    ADC1->SQR3 = (0 << 0) | (1 << 5) | (2 << 10) | (3 << 15) | (4 << 20) | (5 << 25); // sample channel 10
    /* ADC1->SQR1 = 0; */
    /* ADC1->SQR2 = 0; */
    /* ADC1->SQR3 = 0; */
    ADC1->SMPR1 = 0xffffff;
    ADC1->SMPR2 = 0x3fffffff;
    /* ADC1->SMPR1 = (0b111 << 0); // sample time */

    /* RCC->APB2RSTR &= ~(1 << 14); */
    DMA->IFCR |= 0x0fffffff;  // clear all teh interruptz
    static int16_t adcbuf[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 10; ++i) {
        adcbuf[i] = 0;
    }
    adcbuf[10] = 0xadde;

    DMA_Channel1->CCR &= ~(1 << 0);
    DMA_Channel1->CPAR = (size_t)&ADC1->DR;
    DMA_Channel1->CMAR = (DMA_Channel1->CNDTR & 0xffff0000)  // keep reserved bits
        | (size_t)&adcbuf[0]
        ;
    DMA_Channel1->CNDTR = 10;
    DMA_Channel1->CCR = (DMA_Channel1->CCR & 0xffff8000)
        | (0b11 << 12) // high prio
        | (0b01 << 10) // 16 bit memory size
        | (0b01 << 8) // 16 bit peripherial size
        | (1 << 7) // memory inc
        | (1 << 5) // circular mode
        ;
    DMA_Channel1->CCR |= (1 << 0);

    ADC1->CR2 |= (1 << 0); // enable

    DMA_Channel4->CCR &= ~(1 << 0); // channel disable
    DMA_Channel4->CPAR = (size_t)&USART1->DR; // destination register
    DMA_Channel4->CMAR = (size_t)&adcbuf[0]; // source buffer
    DMA_Channel4->CNDTR =
        (DMA_Channel4->CNDTR & 0xffff0000)  // keep reserved bits
        | (sizeof(adcbuf)) // set the transfer size
        ;
    DMA_Channel4->CCR = (DMA_Channel4->CCR & 0xffff8000)
        | (0b01 << 12) // medium prio
        | (1 << 7) // memory inc
        | (1 << 5) // circular mode
        | (1 << 4) // read from memory
        ;
    USART1->SR &= ~(1 << 6); // clear Transmission Complete flag
    delay();
    DMA_Channel4->CCR |=
        (1 << 0) // channel enable
        ;

    GPIOC->BSRR = 1<<8;
    while (1) {
        for (int i = 0; i < 10; ++i) {
            if (i) {
                GPIOC->BSRR = 1<<8;
            } else {
                GPIOC->BSRR = 1<<24;
            }
            /* adcbuf[0] = ADC1->DR; */
            delay();
        }
    }

    return 0;
}
