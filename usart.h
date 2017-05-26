#ifndef USART_H
#define USART_H

#include <array>

#include <stm32f10x.h>

#include "coroutine.h"
#include "notify.h"

extern "C" {
void USART1_IRQHandler();
void USART2_IRQHandler();
void USART3_IRQHandler();
void DMA1_Channel2_IRQHandler();
void DMA1_Channel3_IRQHandler();
}

using usart_rx_data_callback_t = void(*)(const uint8_t ch, const uint16_t sr);
using usart_rx_done_callback_t = void(*)(bool success);

class USART
{
public:
    static constexpr std::size_t MAX_SENDV_POINTERS = 3;
    struct sendv_item
    {
        sendv_item():
            buf(nullptr),
            len(0)
        {

        }

        sendv_item(const uint8_t *buf, uint16_t len):
            buf(buf),
            len(len)
        {

        }

        const uint8_t *buf;
        uint16_t len;
    };

    using sendv_array_t = std::array<sendv_item, MAX_SENDV_POINTERS>;

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
    const IRQn_Type m_tx_dma_irq;
    const IRQn_Type m_rx_dma_irq;
    notifier_t m_tx_idle_notify;
    uint8_t m_tx_sendv_item;
    uint16_t m_tx_offset;
    sendv_array_t m_tx_sendv;

    usart_rx_data_callback_t m_rx_data_cb;
    usart_rx_done_callback_t m_rx_done_cb;

private:
    inline void start_async_tx()
    {
        if (m_tx_dma) {
            m_tx_dma->CMAR = (uint32_t)m_tx_sendv[0].buf;
            m_tx_dma->CNDTR = m_tx_sendv[0].len;
            m_tx_dma->CCR |= DMA_CCR1_EN;
            m_usart->CR3 |= USART_CR3_DMAT;
            m_usart->CR1 |= USART_CR1_TE;
        } else {
            m_usart->CR1 |= USART_CR1_TE | USART_CR1_TXEIE | USART_CR1_TCIE;
        }
    }

public:
    void init(uint32_t baudrate, bool use_cts = false, bool use_rts = false);
    void set_baudrate(const uint32_t baudrate);
    uint16_t calc_brr(const uint32_t baudrate);
    void set_brr(const uint16_t brr);
    void enable();
    void disable();
    ASYNC_CALLABLE tx_ready();
    void send_a(const uint8_t *buf, const uint16_t len);
    ASYNC_CALLABLE send_c(const uint8_t *buf, const uint16_t len);
    ASYNC_CALLABLE sendv_c(const sendv_array_t &bufs);
    void send(const uint8_t *buf, const uint16_t len);
    void sendstr(const char *buf);
    void sendch(const uint8_t ch);

    void set_rx_callback(usart_rx_data_callback_t cb);

    /**
     * Setup a receive buffer for reception via DMA.
     *
     * This is only available for USARTS which have DMA enabled.
     *
     * @param buf Buffer to write to.
     * @param len Length of the buffer.
     */
    void recv_a(uint8_t *buf, const uint16_t len,
                usart_rx_done_callback_t cb);

    template <USART *usart_obj>
    friend void irq_handler();
    template <USART *usart_obj, uint32_t channel, uint32_t channel_shift>
    friend void dma_irq_tx_handler();
    template <USART *usart_obj, uint32_t channel, uint32_t channel_shift>
    friend void dma_irq_rx_handler();
};

extern USART usart1;
extern USART usart2;
extern USART usart3;

#endif // USART_H
