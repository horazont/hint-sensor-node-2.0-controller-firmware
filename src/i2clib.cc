#include "i2clib.h"

#include "utils.h"

I2C i2c1(I2C1, DMA1_Channel6, DMA1_Channel7);
I2C i2c2(I2C2, DMA1_Channel4, DMA1_Channel5);

void i2c1_workaround_reset()
{
    if (!(I2C1->SR2 & I2C_SR2_BUSY)) {
        // no workaround needed
        return;
    }
    i2c1.disable();

    // configure as general-purpose open-drain outputs
    GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7 |
                                 GPIO_CRL_CNF6 | GPIO_CRL_CNF7))
        | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_0
        | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_0;

    GPIOB->BSRR = GPIO_BSRR_BR6;
    // wait for line to go low
    while (GPIOB->IDR & GPIO_IDR_IDR6);

    GPIOB->BSRR = GPIO_BSRR_BR7;
    // wait for line to go low
    while (GPIOB->IDR & GPIO_IDR_IDR7);

    GPIOB->BSRR = GPIO_BSRR_BS6;
    // wait for line to go high
    while (!(GPIOB->IDR & GPIO_IDR_IDR6));

    GPIOB->BSRR = GPIO_BSRR_BS7;
    // wait for line to go high
    while (!(GPIOB->IDR & GPIO_IDR_IDR7));

    i2c1.init();
    i2c1.enable();
}

IRQn_Type I2C::get_ev_irqn(const I2C_TypeDef * const i2c)
{
    if (i2c == I2C1) {
        return I2C1_EV_IRQn;
    } else if (i2c == I2C2) {
        return I2C2_EV_IRQn;
    }
    return NonMaskableInt_IRQn;
}

IRQn_Type I2C::get_er_irqn(const I2C_TypeDef * const i2c)
{
    if (i2c == I2C1) {
        return I2C1_ER_IRQn;
    } else if (i2c == I2C2) {
        return I2C2_ER_IRQn;
    }
    return NonMaskableInt_IRQn;
}


static void short_delay() {
    for (uint32_t i = 0; i < 100; ++i) {
        __asm__ volatile("nop");
    }
}


I2C::I2C(I2C_TypeDef *bus,
         DMA_Channel_TypeDef *tx_dma,
         DMA_Channel_TypeDef *rx_dma):
    m_i2c(bus),
    m_tx_dma(tx_dma),
    m_rx_dma(rx_dma),
    m_ev_irq(get_ev_irqn(bus)),
    m_er_irq(get_er_irqn(bus)),
    m_tx_dma_irq(get_dma_irqn(tx_dma)),
    m_rx_dma_irq(get_dma_irqn(rx_dma))
{

}


void I2C::init()
{
    m_is_busy = false;

    m_i2c->CR1 = I2C_CR1_SWRST;
    short_delay();
    m_i2c->CR1 = 0;

    m_i2c->CCR = 0
        | 180;
    m_i2c->TRISE = 36+1;

    m_i2c->CR2 = 0
        | I2C_CR2_ITBUFEN
        | I2C_CR2_ITEVTEN
        | I2C_CR2_ITERREN
        | 36  // frequency of APB1 domain, in MHz
        ;

    // prepare DMA channel
    m_tx_dma->CCR = 0;
    m_tx_dma->CCR = 0
            // priority level Very High
            | DMA_CCR1_PL_0 | DMA_CCR1_PL_1
            // memory and peripherial data size 8 bits
            // memory increment mode enabled
            | DMA_CCR1_MINC
            // peripherial increment mode disabled
            // circular mode disabled
            // write to peripherial / read from memory
            | DMA_CCR1_DIR
            // enable full transfer and failed transfer interrupts
            | DMA_CCR1_TCIE | DMA_CCR1_TEIE
            // do not enable channel yet
            ;
    m_tx_dma->CPAR = (uint32_t)&m_i2c->DR;

    // prepare DMA channel
    m_rx_dma->CCR = 0;
    m_rx_dma->CCR = 0
            // priority level Very High
            | DMA_CCR1_PL_0 | DMA_CCR1_PL_1
            // memory and peripherial data size 8 bits
            // memory increment mode enabled
            | DMA_CCR1_MINC
            // peripherial increment mode disabled
            // circular mode disabled
            // read from peripherial / write to memory
            // enable full transfer and failed transfer interrupts
            | DMA_CCR1_TCIE | DMA_CCR1_TEIE
            // do not enable channel yet
            ;
    m_rx_dma->CPAR = (uint32_t)&m_i2c->DR;
}

void I2C::enable()
{
    m_i2c->CR1 |= I2C_CR1_PE;
    NVIC_EnableIRQ(m_ev_irq);
    NVIC_EnableIRQ(m_er_irq);
    NVIC_EnableIRQ(m_tx_dma_irq);
    NVIC_EnableIRQ(m_rx_dma_irq);
}

void I2C::disable()
{
    NVIC_DisableIRQ(m_rx_dma_irq);
    NVIC_DisableIRQ(m_tx_dma_irq);
    NVIC_DisableIRQ(m_er_irq);
    NVIC_DisableIRQ(m_ev_irq);
    m_i2c->CR1 &= ~I2C_CR1_PE;
}

void I2C::_prep_smbus_read(const uint8_t device_address,
                           const uint8_t register_address,
                           const uint8_t nbytes,
                           uint8_t *buf)
{
    m_curr_task.device_address = device_address;
    m_curr_task.write_task = false;
    m_curr_task.register_address = register_address;
    m_curr_task.nbytes = nbytes;
    m_curr_task.offset = 0;
    m_curr_task.buf.w = buf;
    m_curr_task.state = I2C_STATE_SELECT_REGISTER;
    m_curr_task.notify.reset();

    if (nbytes > 1) {
        // we don’t use DMA for transfers of 1 byte or less
        m_rx_dma->CCR &= ~DMA_CCR1_EN;
        m_rx_dma->CMAR = (uint32_t)buf;
        m_rx_dma->CNDTR = nbytes;
        m_rx_dma->CCR |= DMA_CCR1_EN;
    }

    m_i2c->CR2 = (m_i2c->CR2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
    m_i2c->CR1 |= I2C_CR1_START;
}

void I2C::_prep_smbus_write(const uint8_t device_address,
                            const uint8_t register_address,
                            const uint8_t nbytes,
                            const uint8_t *buf)
{
    m_curr_task.device_address = device_address;
    m_curr_task.write_task = true;
    m_curr_task.register_address = register_address;
    m_curr_task.nbytes = nbytes;
    m_curr_task.offset = 0;
    m_curr_task.buf.r = buf;
    m_curr_task.state = I2C_STATE_SELECT_REGISTER;
    m_curr_task.notify.reset();

    m_tx_dma->CCR &= ~DMA_CCR1_EN;
    m_tx_dma->CMAR = (uint32_t)buf;
    m_tx_dma->CNDTR = nbytes;
    m_tx_dma->CCR |= DMA_CCR1_EN;

    m_i2c->CR2 = (m_i2c->CR2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
    m_i2c->CR1 |= I2C_CR1_START;
}

void I2C::smbus_read(
        const uint8_t device_address,
        const uint8_t register_address,
        const uint8_t nbytes,
        uint8_t *buf)
{
    _prep_smbus_read(device_address, register_address, nbytes, buf);
    m_curr_task.notify.wait_for_ready();
}

void I2C::smbus_write(
        const uint8_t device_address,
        const uint8_t register_address,
        const uint8_t nbytes,
        const uint8_t *buf)
{
    _prep_smbus_write(device_address, register_address, nbytes, buf);
    m_curr_task.notify.wait_for_ready();
}

bool I2C::smbus_read_a(const uint8_t device_address,
                       const uint8_t register_address,
                       const uint8_t nbytes,
                       uint8_t *buf)
{
    _prep_smbus_read(device_address, register_address, nbytes, buf);
    return true;
}

bool I2C::smbus_write_a(const uint8_t device_address,
                        const uint8_t register_address,
                        const uint8_t nbytes,
                        const uint8_t *buf)
{
    _prep_smbus_write(device_address, register_address, nbytes, buf);
    return true;
}

ASYNC_CALLABLE I2C::smbus_readc(
        const uint8_t device_address,
        const uint8_t register_address,
        const uint8_t nbytes,
        uint8_t *buf)
{
    _prep_smbus_read(device_address, register_address, nbytes, buf);
    return m_curr_task.notify.ready_c();
}

ASYNC_CALLABLE I2C::smbus_writec(
        const uint8_t device_address,
        const uint8_t register_address,
        const uint8_t nbytes,
        const uint8_t *buf)
{
    _prep_smbus_write(device_address, register_address, nbytes, buf);
    return m_curr_task.notify.ready_c();
}


template <I2C *i2c_obj>
static inline void ev_irq_handler()
{
    I2C_TypeDef *const i2c = i2c_obj->m_i2c;
    I2C::i2c_task &curr_task = i2c_obj->m_curr_task;
    const uint8_t sr1 = i2c->SR1;
    if (sr1 & I2C_SR1_SB) {
//        USART2->DR = 's';
        // start generated
        if (curr_task.state == I2C::I2C_STATE_SELECT_REGISTER) {
            i2c->DR = curr_task.device_address << 1;
        } else {
            i2c->DR = curr_task.device_address << 1
                | (curr_task.write_task ? 0 : 1);
        }
        i2c->CR1 |= I2C_CR1_ACK;
    } else if (sr1 & I2C_SR1_ADDR) {
//        USART2->DR = 'a';
        // address generated
        // find out whether we were about to send or to receive
        const uint8_t sr2 = i2c->SR2;
        if (!(sr2 & I2C_SR2_TRA)) {
            if (curr_task.nbytes > 1) {
//                USART2->DR = 'R';
                // receiver mode
                // fire up DMA mode
                const uint8_t cr2 = i2c->CR2;
                i2c->CR2 = (cr2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_DMAEN | I2C_CR2_LAST;
            } else {
//                USART2->DR = '1';
                const uint8_t cr1 = i2c->CR1;
                i2c->CR1 = (cr1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
            }
        }
    } else if (sr1 & I2C_SR1_TXE) {
        if (curr_task.state == I2C::I2C_STATE_SELECT_REGISTER) {
            uint8_t register_address = curr_task.register_address;
            if (curr_task.nbytes > 1) {
                register_address |= 0x80;
            }
            i2c->DR = register_address;
            if (curr_task.write_task) {
                // continue in DMA mode
//                USART2->DR = '|';
                const uint8_t cr2 = i2c->CR2;
                i2c->CR2 = (cr2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_DMAEN;
            } else {
                // repeated start condition for the read part
                i2c->CR1 |= I2C_CR1_START;
//                USART2->DR = 'r';
            }
            curr_task.state = I2C::I2C_STATE_TRANSFER_DATA;
        } else if (curr_task.write_task) {
            // USART2->DR = 't';
            i2c->CR1 |= I2C_CR1_STOP;
            curr_task.notify.trigger();
        }
    } else if (sr1 & I2C_SR1_RXNE) {
        if (curr_task.nbytes == 1) {
            // USART2->DR = '<';
            curr_task.nbytes = 0;
            curr_task.buf.w[0] = i2c->DR;
            curr_task.notify.trigger();
        } else {
            __asm__ volatile ("bkpt #06");
        }
    } else if (sr1 & I2C_SR1_BTF) {
//        USART2->DR = 'f';
    }
}

template <I2C *i2c_obj>
static inline void er_irq_handler()
{
    I2C_TypeDef *const i2c = i2c_obj->m_i2c;
    const uint16_t sr1 = i2c->SR1;
    if (sr1 & I2C_SR1_TIMEOUT) {
        USART2->DR = 'T';
    } else if (sr1 & I2C_SR1_PECERR) {
        USART2->DR = 'P';
    } else if (sr1 & I2C_SR1_SMBALERT) {
        USART2->DR = 'S';
    } else if (sr1 & I2C_SR1_ARLO) {
        USART2->DR = 'l';
    } else {
        USART2->DR = nybble_to_hex(sr1 >> 8);
    }
    i2c->SR1 = 0;
    i2c->CR1 |= I2C_CR1_STOP;
}

template <I2C *i2c_obj, uint32_t channel, uint32_t channel_shift>
void dma_tx_irq_handler()
{
    const uint32_t flags = DMA1->ISR;
    if (flags & (DMA_ISR_TCIF1 << channel_shift)) {
//        USART2->DR = 'D';
    } else if (flags & (DMA_ISR_TEIF1 << channel_shift)) {
//        USART2->DR = 'E';
    }
    // clear all channel 6 interrupts
    DMA1->IFCR = (DMA_IFCR_CHTIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1) << channel_shift;
}

template <I2C *i2c_obj, uint32_t channel, uint32_t channel_shift>
void dma_rx_irq_handler()
{
    I2C::i2c_task &curr_task = i2c_obj->m_curr_task;
    // for RX, I2C takes care of the STOP condition by itself
    const uint32_t flags = DMA1->ISR;
    if (flags & (DMA_ISR_TCIF1 << channel_shift)) {
        curr_task.notify.trigger();
    } else if (flags & (DMA_ISR_TEIF1 << channel_shift)) {
        curr_task.notify.trigger();
    }
    // clear all channel 7 interrupts
    DMA1->IFCR = (DMA_IFCR_CHTIF1 | DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1) << channel_shift;
}

void I2C1_EV_IRQHandler()
{
    ev_irq_handler<&i2c1>();
}

void I2C1_ER_IRQHandler()
{
    er_irq_handler<&i2c1>();
}

void I2C2_EV_IRQHandler()
{
    ev_irq_handler<&i2c2>();
}

void I2C2_ER_IRQHandler()
{
    er_irq_handler<&i2c2>();
}

void DMA1_Channel4_IRQHandler()
{
    dma_tx_irq_handler<&i2c2, (uint32_t)DMA1_Channel4, 12>();
}

void DMA1_Channel5_IRQHandler()
{
    dma_rx_irq_handler<&i2c2, (uint32_t)DMA1_Channel5, 16>();
}

void DMA1_Channel6_IRQHandler()
{
    dma_tx_irq_handler<&i2c1, (uint32_t)DMA1_Channel6, 20>();
}

void DMA1_Channel7_IRQHandler()
{
    dma_rx_irq_handler<&i2c1, (uint32_t)DMA1_Channel7, 24>();
}
