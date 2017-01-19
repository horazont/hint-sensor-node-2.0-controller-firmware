#ifndef USART_H
#define USART_H

#include <stm32f10x.h>

#include "coroutine.h"

extern "C" {
void USART1_IRQHandler();
void USART2_IRQHandler();
void USART3_IRQHandler();
}

class USART
{
public:
    USART() = delete;
    explicit USART(
            USART_TypeDef *usart,
            DMA_Channel_TypeDef *tx_dma,
            DMA_Channel_TypeDef *rx_dma);

private:
    static IRQn_Type get_irqn(USART_TypeDef *usart);

private:
    USART_TypeDef *const m_usart;
    DMA_Channel_TypeDef *const m_tx_dma;
    DMA_Channel_TypeDef *const m_rx_dma;
    const IRQn_Type m_irq;
    volatile uint8_t m_idle_notify;
    const uint8_t *m_tx_buf;
    uint16_t m_tx_len;
    uint16_t m_tx_offset;

public:
    void init(uint32_t baudrate, bool use_modem);
    void enable();
    void disable();
    ASYNC_CALLABLE tx_ready();
    ASYNC_CALLABLE send_c(const uint8_t *buf, const uint16_t len);
    void send(const uint8_t *buf, const uint16_t len);
    void sendstr(const char *buf);
    void sendch(const uint8_t ch);

    template <USART *usart_obj>
    friend void irq_handler();
};

extern USART usart1;
extern USART usart2;
extern USART usart3;

#endif // USART_H
