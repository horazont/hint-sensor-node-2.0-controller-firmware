#include "usart.h"

#include "config.h"


USART usart1(USART1, nullptr, nullptr);
USART usart2(USART2, nullptr, nullptr);
USART usart3(USART3, DMA1_Channel2, DMA1_Channel3);


USART::USART(USART_TypeDef *usart,
             DMA_Channel_TypeDef *tx_dma,
             DMA_Channel_TypeDef *rx_dma):
    m_usart(usart),
    m_tx_dma(tx_dma),
    m_rx_dma(rx_dma),
    m_irq(get_irqn(usart)),
    m_tx_dma_irq(get_dma_irqn(tx_dma)),
    m_rx_dma_irq(get_dma_irqn(rx_dma))
{
    if (m_tx_dma) {
        m_tx_dma->CCR = 0;
        m_tx_dma->CCR = 0
                // peripherial-to-memory mode
                // high priority
                | DMA_CCR1_PL_1
                // byte memory size
                // byte peripherial size
                // enable memory increment mode
                | DMA_CCR1_MINC
                // disable peripherial increment mode
                // disable circular mode
                // read from memory
                | DMA_CCR1_DIR
                // enable full transfer and failed transfer interrupts
                | DMA_CCR1_TCIE | DMA_CCR1_TEIE
                // do not enable channel yet
                ;

        m_tx_dma->CPAR = (uint32_t)&usart->DR;
    }
    if (m_rx_dma) {
        m_rx_dma->CCR = 0;
        m_rx_dma->CCR = 0
                // peripherial-to-memory mode
                // high priority
                | DMA_CCR1_PL_1
                // byte memory size
                // byte peripherial size
                // enable memory increment mode
                | DMA_CCR1_MINC
                // disable peripherial increment mode
                // disable circular mode
                // read from peripherial
                // enable full transfer and failed transfer interrupts
                | DMA_CCR1_TCIE | DMA_CCR1_TEIE
                // do not enable channel yet
                ;
        m_rx_dma->CPAR = (uint32_t)&usart->DR;
    }
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

IRQn_Type USART::get_dma_irqn(DMA_Channel_TypeDef *dmach)
{
    if (dmach == DMA1_Channel1) {
        return DMA1_Channel1_IRQn;
    } else if (dmach == DMA1_Channel2) {
        return DMA1_Channel2_IRQn;
    } else if (dmach == DMA1_Channel3) {
        return DMA1_Channel3_IRQn;
    } else if (dmach == DMA1_Channel4) {
        return DMA1_Channel4_IRQn;
    } else if (dmach == DMA1_Channel5) {
        return DMA1_Channel5_IRQn;
    } else if (dmach == DMA1_Channel6) {
        return DMA1_Channel6_IRQn;
    } else if (dmach == DMA1_Channel7) {
        return DMA1_Channel7_IRQn;
    } else {
        return NonMaskableInt_IRQn;
    }
}

void USART::init(uint32_t baudrate, bool use_cts, bool use_rts)
{
    // disable everything first
    m_usart->CR1 = 0;
    m_usart->CR2 = 0;
    // clear them bits
    m_usart->SR = 0;
    uint32_t cr3 = 0;
    if (use_cts) {
        cr3 |= USART_CR3_CTSE;
    }
    if (use_rts) {
        cr3 |= USART_CR3_RTSE;
    }
    m_usart->CR3 = cr3;
    set_baudrate(baudrate);

}

void USART::set_baudrate(const uint32_t baudrate)
{
    m_usart->BRR = calc_brr(baudrate);
}

uint16_t USART::calc_brr(const uint32_t baudrate)
{
    const uint32_t f_clk = (m_usart == USART1 ? CPU_FREQ : CPU_FREQ / 2);
    // we want to round up. better be below the target baudrate than above
    return (f_clk+baudrate-1) / baudrate;
}

void USART::set_brr(const uint16_t brr)
{
    m_usart->BRR = brr;
}

void USART::enable()
{
    m_usart->CR1 |= USART_CR1_UE;
    NVIC_EnableIRQ(m_irq);
    if (m_tx_dma) {
        NVIC_EnableIRQ(m_tx_dma_irq);
    }
    if (m_rx_dma) {
        NVIC_EnableIRQ(m_rx_dma_irq);
    }
}

void USART::disable()
{
    if (m_rx_dma) {
        NVIC_DisableIRQ(m_rx_dma_irq);
    }
    if (m_tx_dma) {
        NVIC_DisableIRQ(m_tx_dma_irq);
    }
    NVIC_DisableIRQ(m_irq);
    m_usart->CR1 &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
}

ASYNC_CALLABLE USART::tx_ready()
{
    if (m_usart->SR & USART_SR_TC) {
        return WakeupCondition::none();
    }
    return m_tx_idle_notify.ready_c();
}

void USART::send_a(const uint8_t *buf, const uint16_t len)
{
    m_tx_sendv[0] = sendv_item(buf, len);
    m_tx_sendv[1] = sendv_item();
    m_tx_offset = 0;
    m_tx_sendv_item = 0;
    m_tx_idle_notify.reset();

    start_async_tx();
}

ASYNC_CALLABLE USART::send_c(const uint8_t *buf, const uint16_t len)
{
    if (len == 0) {
        return WakeupCondition::none();
    }

    send_a(buf, len);

    return m_tx_idle_notify.ready_c();
}

ASYNC_CALLABLE USART::sendv_c(const USART::sendv_array_t &bufs)
{
    m_tx_sendv = bufs;
    m_tx_offset = 0;
    m_tx_sendv_item = 0;
    m_tx_idle_notify.reset();

    start_async_tx();

    return m_tx_idle_notify.ready_c();
}

void USART::send(const uint8_t *buf, const uint16_t len)
{
    auto &sr = m_usart->SR;
    auto &dr = m_usart->DR;
    auto &cr1 = m_usart->CR1;
    m_tx_idle_notify.reset();
    cr1 |= USART_CR1_TE;
    for (uint16_t i = 0; i < len; ++i) {
        dr = *buf++;
        while (!(sr & USART_SR_TC));
    }
    cr1 &= ~USART_CR1_TE;
    m_tx_idle_notify.trigger();
}

void USART::sendch(const uint8_t ch)
{
    auto &sr = m_usart->SR;
    auto &dr = m_usart->DR;
    auto &cr1 = m_usart->CR1;
    m_tx_idle_notify.reset();
    cr1 |= USART_CR1_TE;
    while (!(sr & USART_SR_TC));
    dr = ch;
    while (!(sr & USART_SR_TC));
    cr1 &= ~USART_CR1_TE;
    m_tx_idle_notify.trigger();
}

void USART::set_rx_callback(usart_rx_data_callback_t cb)
{
    m_rx_data_cb = cb;
    m_usart->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;
}

void USART::recv_a(uint8_t *buf, const uint16_t len,
                   usart_rx_done_callback_t cb)
{
    m_rx_dma->CCR &= ~DMA_CCR1_EN;
    m_rx_dma->CMAR = (uint32_t)buf;
    m_rx_dma->CNDTR = len;
    m_rx_dma->CCR |= DMA_CCR1_EN;

    // disable RX interrupt
    m_usart->CR1 = (m_usart->CR1 & ~(USART_CR1_RXNEIE));
    m_usart->CR3 |= USART_CR3_DMAR;
    m_rx_done_cb = cb;
}

void USART::sendstr(const char *buf)
{
    auto &sr = m_usart->SR;
    auto &dr = m_usart->DR;
    auto &cr1 = m_usart->CR1;
    m_tx_idle_notify.reset();
    cr1 |= USART_CR1_TE;
    while (*buf != 0) {
        while (!(sr & USART_SR_TC));
        dr = *buf++;
    }
    while (!(sr & USART_SR_TC));
    cr1 &= ~USART_CR1_TE;
    m_tx_idle_notify.trigger();
}


template <USART *usart_obj>
static inline void irq_handler()
{
    USART_TypeDef *const usart = usart_obj->m_usart;
    const uint16_t sr = usart->SR;
    if (sr & USART_SR_TXE) {
        const auto &sendv = usart_obj->m_tx_sendv;
        const auto sendv_item = usart_obj->m_tx_sendv_item;
        const auto len = sendv[sendv_item].len;
        if (sendv_item < USART::MAX_SENDV_POINTERS && usart_obj->m_tx_offset < len) {
            usart->DR = sendv[sendv_item].buf[usart_obj->m_tx_offset++];
            if (usart_obj->m_tx_offset == len) {
                usart_obj->m_tx_offset = 0;
                usart_obj->m_tx_sendv_item++;
            }
        } else if (sr & USART_SR_TC) {
            // clear the TC bit
            usart->CR1 &= ~(USART_CR1_TE | USART_CR1_TXEIE | USART_CR1_TCIE);
            usart->SR = ~USART_SR_TC;
            usart_obj->m_tx_idle_notify.trigger();
        }
    }
    if (sr & USART_SR_RXNE) {
        usart_rx_data_callback_t cb = usart_obj->m_rx_data_cb;
        const uint8_t dr = usart->DR;
        if (cb != nullptr) {
            cb(dr, sr);
        }
    }
}

template <USART *usart_obj, uint32_t channel_addr, uint32_t channel_shift>
static inline void dma_irq_tx_handler()
{
    DMA_Channel_TypeDef &channel = *(DMA_Channel_TypeDef*)channel_addr;
    const uint32_t sr = DMA1->ISR;
    if (sr & (DMA_ISR_TCIF1 << channel_shift)) {
        const auto &sendv = usart_obj->m_tx_sendv;
        auto &sendv_item = usart_obj->m_tx_sendv_item;
        sendv_item++;
        const auto &item = sendv[sendv_item];
        if (sendv_item < USART::MAX_SENDV_POINTERS && item.len > 0) {
            // there is another channel, lets proceed
            // disable TC bit by writing a 0 to it
            usart_obj->m_usart->SR = ~USART_SR_TC;
            // disable DMA channel before changing parameters
            channel.CCR &= ~DMA_CCR1_EN;
            channel.CMAR = (uint32_t)item.buf;
            channel.CNDTR = item.len;
            channel.CCR |= DMA_CCR1_EN;
        } else {
            // we need to get notified on TC
            usart_obj->m_usart->CR1 |= USART_CR1_TCIE;
            usart_obj->m_usart->CR3 &= ~(USART_CR3_DMAT);
            channel.CCR &= ~DMA_CCR1_EN;
        }
    } else if (sr & (DMA_ISR_TEIF1 << channel_shift)) {

    }
    DMA1->IFCR = (DMA_IFCR_CHTIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1) << channel_shift;
}

template <USART *usart_obj, uint32_t channel_addr, uint32_t channel_shift>
static inline void dma_irq_rx_handler()
{
    DMA_Channel_TypeDef &channel = *(DMA_Channel_TypeDef*)channel_addr;
    const uint32_t sr = DMA1->ISR;
    if (sr & (DMA_ISR_TCIF1 << channel_shift)) {
        usart_obj->m_usart->CR1 |= USART_CR1_RXNEIE;
        usart_obj->m_usart->CR3 &= ~(USART_CR3_DMAR);
        channel.CCR &= ~DMA_CCR1_EN;
        usart_obj->m_rx_done_cb(true);
    } else if (sr & (DMA_ISR_TEIF1 << channel_shift)) {
        usart_obj->m_usart->CR1 |= USART_CR1_RXNEIE;
        usart_obj->m_usart->CR3 &= ~(USART_CR3_DMAR);
        channel.CCR &= ~DMA_CCR1_EN;
        usart_obj->m_rx_done_cb(false);
    }
    DMA1->IFCR = (DMA_IFCR_CHTIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1) << channel_shift;
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

void DMA1_Channel2_IRQHandler()
{
    // USART3 TX
    dma_irq_tx_handler<&usart3, (uint32_t)DMA1_Channel2, 4>();
}

void DMA1_Channel3_IRQHandler()
{
    // USART3 RX
    dma_irq_rx_handler<&usart3, (uint32_t)DMA1_Channel3, 8>();
}
