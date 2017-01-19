#include "usart.h"

#include "config.h"


USART usart1(USART1, nullptr, nullptr);
USART usart2(USART2, nullptr, nullptr);
USART usart3(USART3, nullptr, nullptr);


USART::USART(USART_TypeDef *usart,
             DMA_Channel_TypeDef *tx_dma,
             DMA_Channel_TypeDef *rx_dma):
    m_usart(usart),
    m_tx_dma(tx_dma),
    m_rx_dma(rx_dma),
    m_irq(get_irqn(usart))
{

}

IRQn_Type USART::get_irqn(USART_TypeDef *usart)
{
    if (usart == USART1) {
        return USART1_IRQn;
    } else if (usart == USART2) {
        return USART2_IRQn;
    } else if (usart == USART3) {
        return USART3_IRQn;
    } else {
        // this will hard-fault when it’s enabled
        return NonMaskableInt_IRQn;
    }
}

void USART::init(uint32_t baudrate, bool use_modem)
{
    const uint32_t f_clk = (m_usart == USART1 ? CPU_FREQ : CPU_FREQ / 2);
    // we want to round up. better be below the target baudrate than above
    const uint16_t brr = (f_clk+baudrate-1) / baudrate;

    // disable everything first
    m_usart->CR1 = 0;
    m_usart->CR2 = 0;
    uint32_t cr3 = 0;
    if (use_modem) {
        cr3 |= USART_CR3_CTSE | USART_CR3_RTSE;
    }
    m_usart->CR3 = cr3;
    m_usart->BRR = brr;

}

void USART::enable()
{
    m_usart->CR1 |= USART_CR1_UE;
    NVIC_EnableIRQ(m_irq);
}

void USART::disable()
{
    NVIC_DisableIRQ(m_irq);
    m_usart->CR1 &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

ASYNC_CALLABLE USART::tx_ready()
{
    if (m_usart->SR & USART_SR_TC) {
        return WakeupCondition::none();
    }
    return WakeupCondition::event(&m_idle_notify);
}

ASYNC_CALLABLE USART::send_c(const uint8_t *buf, const uint16_t len)
{
    if (len == 0) {
        return WakeupCondition::none();
    }

    m_tx_buf = buf;
    m_tx_len = len;
    m_tx_offset = 0;

    m_usart->CR1 |= USART_CR1_TE | USART_CR1_TXEIE;

    return tx_ready();
}

void USART::send(const uint8_t *buf, const uint16_t len)
{
    auto &sr = m_usart->SR;
    auto &dr = m_usart->DR;
    auto &cr1 = m_usart->CR1;
    cr1 |= USART_CR1_TE;
    for (uint16_t i = 0; i < len; ++i) {
        while (!(sr & USART_SR_TC));
        dr = *buf++;
    }
    while (!(sr & USART_SR_TC));
    cr1 &= ~USART_CR1_TE;
}

void USART::sendch(const uint8_t ch)
{
    auto &sr = m_usart->SR;
    auto &dr = m_usart->DR;
    auto &cr1 = m_usart->CR1;
    cr1 |= USART_CR1_TE;
    while (!(sr & USART_SR_TC));
    dr = ch;
    while (!(sr & USART_SR_TC));
    cr1 &= ~USART_CR1_TE;
}

void USART::sendstr(const char *buf)
{
    auto &sr = m_usart->SR;
    auto &dr = m_usart->DR;
    auto &cr1 = m_usart->CR1;
    cr1 |= USART_CR1_TE;
    while (*buf != 0) {
        while (!(sr & USART_SR_TC));
        dr = *buf++;
    }
    while (!(sr & USART_SR_TC));
    cr1 &= ~USART_CR1_TE;
}


template <USART *usart_obj>
static inline void irq_handler()
{
    USART_TypeDef *const usart = usart_obj->m_usart;
    const uint16_t sr = usart->SR;
    if (sr & USART_SR_TXE) {
        if (usart_obj->m_tx_offset == usart_obj->m_tx_len) {
            usart->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE);
        } else {
            usart->DR = usart_obj->m_tx_buf[usart_obj->m_tx_offset++];
        }
    }
}


void USART1_IRQHandler()
{
    irq_handler<&usart1>();
}

void USART2_IRQHandler()
{
    irq_handler<&usart2>();
}

void USART3_IRQHandler()
{
    irq_handler<&usart3>();
}
