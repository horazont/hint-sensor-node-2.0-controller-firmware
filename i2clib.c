#include "i2clib.h"

#include "utils.h"

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
    NVIC_EnableIRQ(31);
    NVIC_EnableIRQ(32);
}

void i2c_disable()
{
    NVIC_DisableIRQ(31);
    NVIC_DisableIRQ(32);
    I2C1->CR1 &= ~I2C_CR1_PE;
}

bool i2c_smbus_read(const uint8_t device_address,
                    const uint8_t register_address,
                    const uint8_t nbytes,
                    uint8_t *buf)
{
    if (is_busy) {
        return false;
    }

    /* is_busy = true; */
    curr_task.device_address = device_address;
    curr_task.write_task = false;
    curr_task.register_address = register_address;
    curr_task.nbytes = nbytes;
    curr_task.offset = 0;
    curr_task.buf.w = buf;
    curr_task.state = I2C_STATE_SELECT_REGISTER;

    I2C1->CR1 |= I2C_CR1_START;
    return true;
}

bool i2c_smbus_write(const uint8_t device_address,
                     const uint8_t register_address,
                     const uint8_t nbytes,
                     const uint8_t *buf)
{
    if (is_busy) {
        return false;
    }

    /* is_busy = true; */
    curr_task.device_address = device_address;
    curr_task.write_task = true;
    curr_task.register_address = register_address;
    curr_task.nbytes = nbytes;
    curr_task.offset = 0;
    curr_task.buf.r = buf;
    curr_task.state = I2C_STATE_SELECT_REGISTER;

    I2C1->CR1 |= I2C_CR1_START;
    return true;
}

void I2C1_EV_IRQHandler()
{
    const uint8_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_SB) {
        USART2->DR = 's';
        // start generated
        if (curr_task.state == I2C_STATE_SELECT_REGISTER) {
            I2C1->DR = curr_task.device_address << 1;
        } else {
            I2C1->DR = curr_task.device_address << 1
                | (curr_task.write_task ? 0 : 1);
        }
        I2C1->CR1 |= I2C_CR1_ACK;
    } else if (sr1 & I2C_SR1_ADDR) {
        // address generated
        // find out whether we were about to send or to receive
        const volatile uint8_t sr2 = I2C1->SR2;
        if (sr2 & I2C_SR2_TRA) {
            USART2->DR = 'S';
        } else {
            // receiver mode
            if (curr_task.nbytes == 1) {
                I2C1->CR1 = (I2C1->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
                USART2->DR = '-';
            } else {
                USART2->DR = 'r';
            }
        }
    } else if (sr1 & I2C_SR1_BTF) {
        USART2->DR = 'f';
    } else if (sr1 & I2C_SR1_TXE) {
        if (curr_task.state == I2C_STATE_SELECT_REGISTER) {
            USART2->DR = 'A';
            uint8_t register_address = curr_task.register_address;
            if (curr_task.nbytes > 1) {
                register_address |= 0x80;
            }
            I2C1->DR = register_address;
            // repeated start condition for the read part
            if (!curr_task.write_task) {
                I2C1->CR1 |= I2C_CR1_START;
            }
            curr_task.state = I2C_STATE_TRANSFER_DATA;
        } else if (curr_task.state == I2C_STATE_TRANSFER_DATA) {
            if (curr_task.write_task) {
                if (curr_task.offset < curr_task.nbytes) {
                    // shift bytes
                    I2C1->DR = curr_task.buf.r[curr_task.offset++];
                    USART2->DR = '>';
                } else {
                    USART2->DR = '.';
                    I2C1->CR1 |= I2C_CR1_STOP;
                }
            }
        } else {
            USART2->DR = '!';
        }
    } else if (sr1 & I2C_SR1_RXNE) {
        curr_task.buf.w[curr_task.offset++] = I2C1->DR;
        if (curr_task.offset == curr_task.nbytes - 1) {
            I2C1->CR1 = (I2C1->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
            USART2->DR = '-';
        } else {
            USART2->DR = '<';
        }
    } else if (sr1 & I2C_SR1_STOPF){
        USART2->DR = ':';
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
