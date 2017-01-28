#include "dht11.h"

#include "notify.h"

#include <stm32f10x.h>

static uint16_t *m_humidity;
static uint16_t *m_temperature;
static uint8_t m_checksum_accum;
static uint8_t m_checksum_recvd;
static bool *m_valid;
static bool m_leaders_allowed;
static notifier_t m_done;
static uint8_t m_state;
static uint8_t m_curr_bit_mask;


void dht11_init()
{
    TIM1->CR1 = 0
            | TIM_CR1_URS;
    TIM1->CR2 = 0;
    TIM1->PSC = 7;
    TIM1->DIER = TIM_DIER_CC2IE;
    TIM1->CCMR1 = 0
            | TIM_CCMR1_CC1S_0
            | TIM_CCMR1_IC1F_1
            | TIM_CCMR1_IC2F_1
            | TIM_CCMR1_CC2S_1;
    TIM1->CCMR2 = 0;
    TIM1->CCER = TIM_CCER_CC2P;
    TIM1->CNT = 0;
    TIM1->SR = 0;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->SMCR = 0
            | TIM_SMCR_MSM
            | TIM_SMCR_TS_0 | TIM_SMCR_TS_2
            | TIM_SMCR_SMS_2
            ;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    NVIC_EnableIRQ(TIM1_CC_IRQn);
}


void dht11_sample(uint16_t &humidity, uint16_t &temperature, bool &valid)
{
    dht11_sample_a(humidity, temperature, valid);
    m_done.wait_for_ready();
}


void dht11_sample_a(uint16_t &humidity, uint16_t &temperature, bool &valid)
{
    dht11_sample_prep(humidity, temperature, valid);
    GPIOB->BSRR = GPIO_BSRR_BR5;
    TIM1->CR1 |= TIM_CR1_CEN;
    for (unsigned int i = 0; i < 20000; ++i) {
        __asm__ volatile("nop");
    }
    GPIOB->BSRR = GPIO_BSRR_BS5;
}

void dht11_sample_prep(uint16_t &humidity, uint16_t &temperature, bool &valid)
{
    humidity = 0;
    temperature = 0;
    valid = false;

    m_humidity = &humidity;
    m_temperature = &temperature;
    m_valid = &valid;
    m_state = 0;
    m_curr_bit_mask = 128;
    m_checksum_accum = 0;
    m_leaders_allowed = true;
    m_checksum_recvd = 0;

    m_done.reset();

    TIM1->CR1 = 0
            | TIM_CR1_URS;
    TIM1->CR2 = 0;
    TIM1->PSC = 7;
    TIM1->DIER = TIM_DIER_CC2IE;
    TIM1->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;
    TIM1->CCMR2 = 0;
    TIM1->CCER = TIM_CCER_CC2P;
    TIM1->CNT = 0;
    TIM1->SR = 0;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->SMCR = 0
            | TIM_SMCR_MSM
            | TIM_SMCR_TS_0 | TIM_SMCR_TS_2
            | TIM_SMCR_SMS_2
            ;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
}

ASYNC_CALLABLE dht11_sample_c(uint16_t &humidity, uint16_t &temperature, bool &valid)
{
    dht11_sample_a(humidity, temperature, valid);
    return m_done.ready_c();
}

ASYNC_CALLABLE dht11_sample_fire_c()
{
    GPIOB->BSRR = GPIO_BSRR_BS5;
    return m_done.ready_c();
}


static inline bool shift_bitmask()
{
    m_curr_bit_mask >>= 1;
    if (!m_curr_bit_mask) {
        m_curr_bit_mask = 128;
        return true;
    }
    return false;
}


void TIM1_CC_IRQHandler()
{
    const uint16_t ccr1 = TIM1->CCR1;
    (void)ccr1;
    const uint16_t ccr2 = TIM1->CCR2;

    if (ccr2 >= 0x0288) {
        if (!m_leaders_allowed) {
            __asm__ volatile ("bkpt #01");
        }
        // leader bit or something
        return;
    }

    m_leaders_allowed = false;

    const bool bit_value = ccr2 >= 0x01a7;

    switch (m_state) {
    case 0:
    {
        if (bit_value) {
            *m_humidity |= ((uint16_t)m_curr_bit_mask) << 8;
        }

        if (shift_bitmask()) {
            m_checksum_accum += ((*m_humidity) >> 8) & 0xff;
            m_state++;
        }
        break;
    }
    case 1:
    {
        if (bit_value) {
            *m_humidity |= m_curr_bit_mask;
        }

        if (shift_bitmask()) {
            m_checksum_accum += (*m_humidity) & 0xff;
            m_state++;
        }
        break;
    }
    case 2:
    {
        if (bit_value) {
            *m_temperature |= ((uint16_t)m_curr_bit_mask) << 8;
        }

        if (shift_bitmask()) {
            m_checksum_accum += ((*m_temperature) >> 8) & 0xff;
            m_state++;
        }
        break;
    }
    case 3:
    {
        if (bit_value) {
            *m_temperature |= m_curr_bit_mask;
        }

        if (shift_bitmask()) {
            m_checksum_accum += (*m_temperature) & 0xff;
            m_state++;
        }
        break;
    }
    case 4:
    {
        if (bit_value) {
            m_checksum_recvd |= m_curr_bit_mask;
        }
        if (shift_bitmask()) {
            *m_valid = m_checksum_accum == m_checksum_recvd;
            m_done.trigger();
            TIM1->CR1 &= ~TIM_CR1_CEN;
        }
        break;
    }
    default:
        __asm__ volatile("bkpt #01");
    }
}
