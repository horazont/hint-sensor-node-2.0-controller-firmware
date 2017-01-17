#include "i2clib.h"

#include "utils.h"

#define DMA_TX_CHANNEL DMA1_Channel6
#define DMA_RX_CHANNEL DMA1_Channel7

enum i2c_state {
    I2C_STATE_SELECT_REGISTER = 0,
    I2C_STATE_TRANSFER_DATA = 1
};

struct i2c_task {
    uint8_t device_address;
    bool write_task;
    uint8_t register_address;
    uint8_t nbytes;
    uint8_t offset;
    enum i2c_state state;
    volatile uint8_t *notify;

    union {
        uint8_t *w;
        const uint8_t *r;
    } buf;
};

static struct i2c_task curr_task;
static volatile bool is_busy = false;


static void short_delay() {
    for (uint32_t i = 0; i < 100; ++i) {
        __asm__ volatile("nop");
    }
}

void i2c_init()
{
    is_busy = false;

    I2C1->CR1 = I2C_CR1_SWRST;
    short_delay();
    I2C1->CR1 = 0;

    I2C1->CCR = 0
        | 180;
    I2C1->TRISE = 36+1;

    I2C1->CR2 = 0
        | I2C_CR2_ITBUFEN
        | I2C_CR2_ITEVTEN
        | I2C_CR2_ITERREN
        | 36  // frequency of APB1 domain, in MHz
        ;

    GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 |
                                 GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0 |
                                 GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 |
                                 GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0))
        | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0
        | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0
        ;

    // prepare DMA channel
    DMA1_Channel6->CCR = 0
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
    DMA1_Channel6->CPAR = (uint32_t)&I2C1->DR;

    // prepare DMA channel
    DMA1_Channel7->CCR = 0
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
    DMA1_Channel7->CPAR = (uint32_t)&I2C1->DR;
}

void i2c_workaround_reset()
{
    i2c_disable();

    // configure as general-purpose open-drain outputs
    GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0 |
                                 GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0 |
                                 GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 |
                                 GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0))
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

    i2c_init();
    i2c_enable();
}

void i2c_enable()
{
    I2C1->CR1 |= I2C_CR1_PE;
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void i2c_disable()
{
    NVIC_DisableIRQ(DMA1_Channel6_IRQn);
    NVIC_DisableIRQ(DMA1_Channel7_IRQn);
    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C2_EV_IRQn);
    I2C1->CR1 &= ~I2C_CR1_PE;
}

static void _i2c_prep_smbus_read(const uint8_t device_address,
                                 const uint8_t register_address,
                                 const uint8_t nbytes,
                                 uint8_t *buf,
                                 volatile uint8_t *notify)
{
    curr_task.device_address = device_address;
    curr_task.write_task = false;
    curr_task.register_address = register_address;
    curr_task.nbytes = nbytes;
    curr_task.offset = 0;
    curr_task.buf.w = buf;
    curr_task.state = I2C_STATE_SELECT_REGISTER;
    curr_task.notify = notify;

    DMA_RX_CHANNEL->CCR &= ~DMA_CCR1_EN;
    DMA_RX_CHANNEL->CMAR = (uint32_t)buf;
    DMA_RX_CHANNEL->CNDTR = nbytes;
    DMA_RX_CHANNEL->CCR |= DMA_CCR1_EN;

    I2C1->CR2 = (I2C1->CR2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
    I2C1->CR1 |= I2C_CR1_START;
}

static void _i2c_prep_smbus_write(const uint8_t device_address,
                                  const uint8_t register_address,
                                  const uint8_t nbytes,
                                  const uint8_t *buf,
                                  volatile uint8_t *notify)
{
    curr_task.device_address = device_address;
    curr_task.write_task = true;
    curr_task.register_address = register_address;
    curr_task.nbytes = nbytes;
    curr_task.offset = 0;
    curr_task.buf.r = buf;
    curr_task.state = I2C_STATE_SELECT_REGISTER;
    curr_task.notify = notify;

    DMA_TX_CHANNEL->CCR &= ~DMA_CCR1_EN;
    DMA_TX_CHANNEL->CMAR = (uint32_t)buf;
    DMA_TX_CHANNEL->CNDTR = nbytes;
    DMA_TX_CHANNEL->CCR |= DMA_CCR1_EN;

    I2C1->CR2 = (I2C1->CR2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN;
    I2C1->CR1 |= I2C_CR1_START;
}

bool i2c_smbus_read(const uint8_t device_address,
                    const uint8_t register_address,
                    const uint8_t nbytes,
                    uint8_t *buf)
{
    _i2c_prep_smbus_read(device_address, register_address, nbytes, buf, nullptr);
    return true;
}

bool i2c_smbus_write(const uint8_t device_address,
                     const uint8_t register_address,
                     const uint8_t nbytes,
                     const uint8_t *buf)
{
    _i2c_prep_smbus_write(device_address, register_address, nbytes, buf, nullptr);
    return true;
}

ASYNC_CALLABLE i2c_smbus_readc(
        const uint8_t device_address,
        const uint8_t register_address,
        const uint8_t nbytes,
        uint8_t *buf,
        volatile uint8_t *notify)
{
    *notify = 1;
    _i2c_prep_smbus_read(device_address, register_address, nbytes, buf, notify);
    return WakeupCondition::event(notify);
}

ASYNC_CALLABLE i2c_smbus_writec(
        const uint8_t device_address,
        const uint8_t register_address,
        const uint8_t nbytes,
        const uint8_t *buf,
        volatile uint8_t *notify)
{
    *notify = 1;
    _i2c_prep_smbus_write(device_address, register_address, nbytes, buf, notify);
    return WakeupCondition::event(notify);
}


void I2C1_EV_IRQHandler()
{
    const uint8_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_SB) {
//        USART2->DR = 's';
        // start generated
        if (curr_task.state == I2C_STATE_SELECT_REGISTER) {
            I2C1->DR = curr_task.device_address << 1;
        } else {
            I2C1->DR = curr_task.device_address << 1
                | (curr_task.write_task ? 0 : 1);
        }
        I2C1->CR1 |= I2C_CR1_ACK;
    } else if (sr1 & I2C_SR1_ADDR) {
//        USART2->DR = 'a';
        // address generated
        // find out whether we were about to send or to receive
        const uint8_t sr2 = I2C1->SR2;
        if (!(sr2 & I2C_SR2_TRA)) {
//            USART2->DR = 'R';
            // receiver mode
            // fire up DMA mode
            const uint8_t cr2 = I2C1->CR2;
            I2C1->CR2 = (cr2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_DMAEN | I2C_CR2_LAST;
        }
    } else if (sr1 & I2C_SR1_TXE) {
        if (curr_task.state == I2C_STATE_SELECT_REGISTER) {
            uint8_t register_address = curr_task.register_address;
            if (curr_task.nbytes > 1) {
                register_address |= 0x80;
            }
            I2C1->DR = register_address;
            if (curr_task.write_task) {
                // continue in DMA mode
//                USART2->DR = '|';
                const uint8_t cr2 = I2C1->CR2;
                I2C1->CR2 = (cr2 & I2C_CR2_FREQ) | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_DMAEN;
            } else {
                // repeated start condition for the read part
                I2C1->CR1 |= I2C_CR1_START;
//                USART2->DR = 'r';
            }
            curr_task.state = I2C_STATE_TRANSFER_DATA;
        } else if (curr_task.write_task) {
            // USART2->DR = 't';
            I2C1->CR1 |= I2C_CR1_STOP;
            if (curr_task.notify) {
                *curr_task.notify = 0;
            }
        }
    } else if (sr1 & I2C_SR1_RXNE) {
        __ASM volatile("bkpt #06");
    } else if (sr1 & I2C_SR1_BTF) {
//        USART2->DR = 'f';
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

void DMA1_Channel6_IRQHandler()
{
    const uint32_t flags = DMA1->ISR;
    if (flags & DMA_ISR_TCIF6) {
//        USART2->DR = 'D';
    } else if (flags & DMA_ISR_TEIF6) {
//        USART2->DR = 'E';
    }
    // clear all channel 6 interrupts
    DMA1->IFCR = DMA_IFCR_CHTIF6 | DMA_IFCR_CGIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CTEIF6;
}

void DMA1_Channel7_IRQHandler()
{
    // for RX, I2C takes care of the STOP condition by itself
    const uint32_t flags = DMA1->ISR;
    if (flags & DMA_ISR_TCIF7) {
        if (curr_task.notify) {
            *curr_task.notify = 0;
        }
    } else if (flags & DMA_ISR_TEIF7) {
        if (curr_task.notify) {
            *curr_task.notify = 0;
        }
    }
    // clear all channel 7 interrupts
    DMA1->IFCR = DMA_IFCR_CHTIF7 | DMA_IFCR_CGIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7;
}
