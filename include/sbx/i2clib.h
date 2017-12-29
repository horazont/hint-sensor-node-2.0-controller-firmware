#ifndef SBX_I2CLIB_H
#define SBX_I2CLIB_H

#include <stdbool.h>

#include <stm32f10x.h>

#include "coroutine.h"
#include "notify.h"


class I2C {
public:
    struct Metrics {
        Metrics();

        uint16_t transaction_overruns;
    };

public:
    I2C() = delete;
    explicit I2C(I2C_TypeDef *bus,
                 DMA_Channel_TypeDef *tx_dma,
                 DMA_Channel_TypeDef *rx_dma);

private:
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
        notifier_t notify;

        union {
            uint8_t *w;
            const uint8_t *r;
        } buf;
    };

    I2C_TypeDef *const m_i2c;
    DMA_Channel_TypeDef *const m_tx_dma;
    DMA_Channel_TypeDef *const m_rx_dma;
    const IRQn_Type m_ev_irq;
    const IRQn_Type m_er_irq;
    const IRQn_Type m_tx_dma_irq;
    const IRQn_Type m_rx_dma_irq;
    i2c_task m_curr_task;
    uint8_t m_last_sr1;
    uint8_t m_last_err;
    Metrics m_metrics;
    bool m_busy;

private:
    static IRQn_Type get_ev_irqn(const I2C_TypeDef *const i2c);
    static IRQn_Type get_er_irqn(const I2C_TypeDef *const i2c);

    inline void finish()
    {
        m_curr_task.notify.trigger();
        m_busy = false;
    }

    void _prep_smbus_read(const uint8_t device_address,
                          const uint8_t register_address,
                          const uint8_t nbytes,
                          uint8_t *buf);
    void _prep_smbus_write(const uint8_t device_address,
                           const uint8_t register_address,
                           const uint8_t nbytes,
                           const uint8_t *buf);

public:
    void init();
    void enable();
    void disable();

    void crude_hack();

    bool is_busy();
    inline uint8_t last_err() const
    {
        return m_last_err;
    }

    void smbus_read(const uint8_t device_address,
                    const uint8_t register_address,
                    const uint8_t nbytes,
                    uint8_t *buf);

    void smbus_write(const uint8_t device_address,
                     const uint8_t register_address,
                     const uint8_t nbytes,
                     const uint8_t *buf);

    bool smbus_read_a(const uint8_t device_address,
                      const uint8_t register_address,
                      const uint8_t nbytes,
                      uint8_t *buf);

    bool smbus_write_a(const uint8_t device_address,
                       const uint8_t register_address,
                       const uint8_t nbytes,
                       const uint8_t *buf);

    ASYNC_CALLABLE smbus_writec(const uint8_t device_address,
                                const uint8_t register_address,
                                const uint8_t nbytes,
                                const uint8_t *buf);

    ASYNC_CALLABLE smbus_readc(const uint8_t device_address,
                               const uint8_t register_address,
                               const uint8_t nbytes,
                               uint8_t *buf);

    inline const Metrics &metrics() const
    {
        return m_metrics;
    }

    template <I2C *usart_obj>
    friend void ev_irq_handler();
    template <I2C *usart_obj>
    friend void er_irq_handler();
    template <I2C *usart_obj, uint32_t channel, uint32_t channel_shift>
    friend void dma_tx_irq_handler();
    template <I2C *usart_obj, uint32_t channel, uint32_t channel_shift>
    friend void dma_rx_irq_handler();
};

void i2c1_workaround_reset();

extern I2C i2c1;
extern I2C i2c2;

#ifdef __cplusplus
extern "C" {
#endif

void I2C1_EV_IRQHandler();
void I2C1_ER_IRQHandler();
void I2C2_EV_IRQHandler();
void I2C2_ER_IRQHandler();
void DMA1_Channel4_IRQHandler();
void DMA1_Channel5_IRQHandler();
void DMA1_Channel6_IRQHandler();
void DMA1_Channel7_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif
