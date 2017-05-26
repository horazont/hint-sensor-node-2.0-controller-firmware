#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <cstring>

#include <stm32f10x.h>

#include "i2clib.h"
#include "clock.h"
#include "utils.h"
#include "scheduler.h"
#include "imu.h"
#include "comm_sbx.h"
#include "comm_esp.h"
#include "usart.h"
#include "lightsensor_freq.h"
#include "onewire.h"

#define STACK_TOP (void*)(0x20002000)

int main();

extern "C" {

void NMI_Handler()
{
    __ASM volatile("bkpt #03");
    while (1);
}

void HardFault_Handler()
{
    __ASM volatile("bkpt #04");
    while (1);
}

void MemManage_Handler()
{
    __ASM volatile("bkpt #05");
    while (1);
}

}


static const uint8_t lightsensor_ch_map[4] = {
    0, 3, 2, 1
};


using CommInterfaceTX = CommESPTX;
using CommInterfaceRX = CommESPRX;


class BlinkLED: public Coroutine
{
public:
    void operator()()
    {
        Coroutine::operator()();
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            await(sleep_c(500, now));
            GPIOA->BSRR = GPIO_BSRR_BS5;
            await(sleep_c(500, now));
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        COROUTINE_END;
    }
};


class SampleLightsensor: public Coroutine
{
public:
    explicit SampleLightsensor(CommInterfaceTX &comm):
        m_comm(comm)
    {

    }

private:
    CommInterfaceTX &m_comm;

    sched_clock::time_point m_last_start;
    CommInterfaceTX::buffer_t::buffer_handle_t m_out_buffer_handle;
    sbx_msg_t *m_out_buffer;
    uint16_t m_out_length;

    unsigned int m_i;
    unsigned int m_j;

public:
    void operator()()
    {
        Coroutine::operator()();
        m_j = 0;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            await(m_comm.buffer().any_buffer_free());
            m_out_buffer_handle = m_comm.buffer().allocate(
                        *((uint8_t**)&m_out_buffer),
                        CommInterfaceTX::buffer_t::BUFFER_SIZE);
            if (m_out_buffer_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER) {
                continue;
            }

            m_out_buffer->type = sbx_msg_type::SENSOR_LIGHT;

            for (m_i = 0; m_i < SBX_LIGHT_SENSOR_SAMPLES; ++m_i) {
                if (m_i > 0) {
                    await(sleep_c(1000, m_last_start));
                }
                m_last_start = now;
                m_out_buffer->payload.light.samples[m_i].timestamp = sched_clock::now_raw();
                for (m_j = 0; m_j < SBX_LIGHT_SENSOR_CHANNELS; ++m_j)
                {
                    ls_freq_select_channel(m_j);
                    await(sleep_c(200));
                    m_out_buffer->payload.light.samples[m_i].ch[lightsensor_ch_map[m_j]] = ls_freq_read();
                }
            }

            m_comm.buffer().set_ready(
                        m_out_buffer_handle,
                        sizeof(sbx_msg_type) + sizeof(sbx_msg_light_t));
        }
        COROUTINE_END;
    }
};


class IMUSensorStream: public Coroutine
{
public:
    IMUSensorStream(CommInterfaceTX &comm):
        Coroutine(),
        m_comm(comm),
        m_hangover_len(0),
        m_out_buffer_handle(CommInterfaceTX::buffer_t::INVALID_BUFFER)
    {
    }

private:
    CommInterfaceTX &m_comm;
    std::array<uint16_t, IMU_BUFFER_LENGTH> m_hangover_buffer;
    uint_fast8_t m_hangover_len;


private:
    const imu_buffer_t *m_src_buffer;
    uint_fast8_t m_src_axis;
    imu_source_t m_src_type;

    CommInterfaceTX::buffer_t::buffer_handle_t m_out_buffer_handle;
    sbx_msg_t *m_out_buffer;
    uint16_t m_out_length;

    uint16_t m_sample_counter;

    bool m_average_initialised;
    uint16_t m_average;

    std::array<uint8_t, SENSOR_STREAM::MAX_BITMAP_SIZE> m_bitmap_buffer;
    std::array<uint8_t, MAX_XBEE_PAYLOAD_SIZE> m_packet_buffer;
    uint8_t m_bitmap_byte;
    uint8_t m_bitmap_bit;

    uint8_t m_payload_size;
    uint8_t m_samples_used;
    uint8_t m_packet_pos;

    bool m_packet_initialised;

    uint8_t m_i;

    inline void reset()
    {
        memset(&m_bitmap_buffer[0], 0, m_bitmap_buffer.size());
        m_bitmap_bit = 7;
        m_bitmap_byte = 0;
        m_payload_size = 0;
        m_samples_used = 0;
        m_packet_pos = 0;
        m_packet_initialised = false;
        m_average_initialised = false;
    }

    inline bool pack_sample(uint16_t sample)
    {
        if (!m_average_initialised) {
            m_average = sample;
            m_average_initialised = true;
            m_sample_counter += 1;
            // we don’t need to code the sample
            return true;
        }

        const uint16_t value = sample - m_average;
        const int16_t svalue = (int16_t)value;

        if (m_bitmap_bit == 7) {
            m_payload_size += 1;
        }

        if (-128 <= svalue && svalue < 127) {
            m_payload_size += 1;
            if (m_payload_size > SENSOR_STREAM::MAX_ENCODED_SAMPLE_BYTES) {
                return false;
            }

            m_packet_buffer[m_packet_pos++] = (uint8_t)(int8_t)value;
            m_bitmap_buffer[m_bitmap_byte] |= (1<<m_bitmap_bit);
        } else {
            m_payload_size += 2;
            if (m_payload_size > SENSOR_STREAM::MAX_ENCODED_SAMPLE_BYTES) {
                return false;
            }

            memcpy(&m_packet_buffer[m_packet_pos], &value, sizeof(uint16_t));
            m_packet_pos += 2;
            // zero bit in bitmap
        }

        m_sample_counter += 1;

        if (m_bitmap_bit == 0) {
            m_bitmap_bit = 7;
            m_bitmap_byte += 1;
        } else {
            m_bitmap_bit -= 1;
        }

        return true;
    }

    inline void finish_packet()
    {
        const uint8_t bitmap_len = (m_bitmap_bit == 7 ? m_bitmap_byte : m_bitmap_byte+1);
        // const uint8_t total_len = bitmap_len + m_packet_pos + 2;
        m_out_buffer->type = static_cast<sbx_msg_type>(
                    static_cast<uint8_t>(sbx_msg_type::SENSOR_STREAM_ACCEL_X) +
                    m_src_type*3 +
                    m_src_axis);
        m_out_buffer->payload.sensor_stream.average = m_average;
        memcpy(&m_out_buffer->payload.sensor_stream.data[0],
                &m_bitmap_buffer[0],
                bitmap_len);
        memcpy(&m_out_buffer->payload.sensor_stream.data[bitmap_len],
               &m_packet_buffer[0],
                m_packet_pos);

        m_comm.buffer().set_ready(
                    m_out_buffer_handle,
                    bitmap_len + m_packet_pos +
                    sizeof(sbx_msg_type) +
                    sizeof(sbx_msg_sensor_stream_t)
                    );
    }

public:
    void operator()(const imu_source_t src, const uint8_t axis)
    {
        Coroutine::operator()();
        m_src_type = src;
        m_src_axis = axis;
        m_out_buffer_handle = CommInterfaceTX::buffer_t::BUFFER_SIZE;
        m_sample_counter = 0;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            // wait for buffer to write into
            await(m_comm.buffer().any_buffer_free());
            m_out_buffer_handle = m_comm.buffer().allocate(
                        *((uint8_t**)&m_out_buffer),
                        CommInterfaceTX::buffer_t::BUFFER_SIZE,
                        PRIO_NO_RETRIES);
            if (m_out_buffer_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER) {
                continue;
            }

            reset();

            // pack data into buffer until full
            do {
                await(imu_timed_full_buffer(m_src_buffer, m_src_type));

                if (!m_packet_initialised) {
                    m_out_buffer->payload.sensor_stream.seq = m_src_buffer->first_sample-m_hangover_len;
                    if (m_hangover_len) {
                        for (m_i = 0; m_i < m_hangover_len; ++m_i) {
                            // hangover buffer is too small to cause a packet to be emitted
                            pack_sample(m_hangover_buffer[m_i]);
                        }
                        m_hangover_len = 0;
                    }
                    m_packet_initialised = true;
                }

                m_samples_used = 0;
                for (m_i = 0; m_i < m_src_buffer->samples.size(); ++m_i)
                {
                    if (!pack_sample(m_src_buffer->samples[m_i].vector[m_src_axis])) {
                        break;
                    }
                    m_samples_used += 1;
                }

            } while (m_samples_used == m_src_buffer->samples.size() &&
                     m_payload_size < SENSOR_STREAM::MAX_ENCODED_SAMPLE_BYTES-3);

            m_hangover_len = 0;
            for (m_i = m_samples_used; m_i < m_src_buffer->samples.size(); ++m_i) {
                m_hangover_buffer[m_i-m_samples_used] =
                        m_src_buffer->samples[m_i].vector[m_src_axis];
                m_hangover_len += 1;
            }

            finish_packet();
        }
        COROUTINE_END;
    }
};


class MiscTask: public Coroutine {
public:
    MiscTask(CommInterfaceTX &tx, CommInterfaceRX &rx):
        m_tx(tx),
        m_rx(rx)
    {

    }

private:
    CommInterfaceTX &m_tx;
    CommInterfaceRX &m_rx;
    sched_clock::time_point m_last_wakeup;
    CommInterfaceTX::buffer_t::buffer_handle_t m_handle;
    sbx_msg_t *m_buf;

public:
    void operator()() {
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            m_last_wakeup = now;
            do {
                await(m_tx.buffer().any_buffer_free());
                m_handle = m_tx.buffer().allocate(
                            *(uint8_t**)&m_buf,
                            sizeof(sbx_msg_type) + sizeof(sbx_msg_status_t)
                            );
            } while (m_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER);
            m_buf->type = sbx_msg_type::STATUS;
            m_buf->payload.status.rtc = stm32_rtc::now_raw();
            m_buf->payload.status.uptime = sched_clock::now_raw();
            m_buf->payload.status.xbee_status.rx_errors = m_rx.errors();
            m_buf->payload.status.xbee_status.rx_overruns = m_rx.overruns();
            m_buf->payload.status.xbee_status.rx_checksum_errors = m_rx.checksum_errors();
            m_buf->payload.status.xbee_status.rx_unknown_frames = m_rx.unknown_frames();
            m_buf->payload.status.xbee_status.rx_skipped_bytes = m_rx.skipped_bytes();
            {
                uint16_t most_allocated;
                m_rx.buffer().fetch_stats_and_reset(
                            most_allocated
                            );
                m_buf->payload.status.xbee_status.rx_buffer_most_allocated = most_allocated;
            }
            m_buf->payload.status.xbee_status.tx_lost = m_rx.tx_lost();
            m_buf->payload.status.xbee_status.tx_retries = m_rx.tx_retries();
            {
                uint16_t most_allocated;
                m_tx.buffer().fetch_stats_and_reset(
                            most_allocated
                            );
                m_buf->payload.status.xbee_status.tx_buffer_most_allocated = most_allocated;
            }
            m_buf->payload.status.core_status.undervoltage_detected = 0;  // TODO
            m_tx.buffer().set_ready(m_handle);
            /*memset(&m_addr, 0, sizeof(onewire_addr_t));
            do {
                await_call(m_find_next, m_addr);
                await(usart2.send_c(&m_addr[0], sizeof(onewire_addr_t)));
            } while (m_find_next.status() == ONEWIRE_PRESENCE);*/
            await(sleep_c(10000, m_last_wakeup));
        }
        COROUTINE_END;
    }
};


static BlinkLED blink;
static OnewireCore onewire(usart1);
static CommInterfaceTX commtx(usart3);
static CommInterfaceRX commrx(usart3);
static IMUSensorStream stream_ax(commtx);
static IMUSensorStream stream_ay(commtx);
static IMUSensorStream stream_az(commtx);
static IMUSensorStream stream_mx(commtx);
static IMUSensorStream stream_my(commtx);
static IMUSensorStream stream_mz(commtx);
static SampleLightsensor sample_lightsensor(commtx);
static MiscTask misc(commtx, commrx);
static Scheduler<12> scheduler;


/*static void dump()
{
    const uint16_t cnt = TIM1->CNT;
    const uint16_t ccr1 = TIM1->CCR1;
    const uint16_t ccr2 = TIM1->CCR2;
    const uint16_t idr_A = GPIOA->IDR;
    const uint16_t idr_B = GPIOB->IDR;
    const uint16_t odr_B = GPIOB->ODR;

    char buf[5];
    buf[4] = ' ';
    uint16_to_hex(cnt, buf);
    usart2.send((const uint8_t*)buf, 5);

    uint16_to_hex(ccr1, buf);
    usart2.send((const uint8_t*)buf, 5);

    uint16_to_hex(ccr2, buf);
    usart2.send((const uint8_t*)buf, 5);

    uint16_to_hex((idr_A & (1<<8)) | ((idr_B & (1<<5)) >> 1) | ((odr_B & (1<<5)) >> 5), buf);

    buf[4] = '\n';
    usart2.send((const uint8_t*)buf, 5);
}*/


int main() {
    // ensure that ADC doesn’t go up in flames
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;

    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_USART2RST | RCC_APB1RSTR_USART3RST;
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_IOPCRST | RCC_APB2RSTR_IOPDRST | RCC_APB2RSTR_IOPERST | RCC_APB2RSTR_TIM1RST | RCC_APB2RSTR_USART1RST | RCC_APB2RSTR_ADC1RST;
    DMA1->IFCR = 0xfffffff;  // clear all the interrupt flags

    // give it a moment to reset
    for (uint32_t i = 0; i < 1000; ++i) {
        __asm__ volatile("nop");
    }

    RCC->APB1RSTR = 0;
    RCC->APB2RSTR = 0;

    RCC->APB2ENR = 0
            | RCC_APB2ENR_IOPAEN
            | RCC_APB2ENR_IOPBEN
            | RCC_APB2ENR_IOPCEN
            | RCC_APB2ENR_IOPDEN
            | RCC_APB2ENR_IOPEEN
            | RCC_APB2ENR_AFIOEN
            | RCC_APB2ENR_USART1EN
            | RCC_APB2ENR_ADC1EN
            | RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR = 0
            | RCC_APB1ENR_PWREN
            | RCC_APB1ENR_BKPEN
            | RCC_APB1ENR_TIM3EN
            | RCC_APB1ENR_USART2EN
            | RCC_APB1ENR_USART3EN
            | RCC_APB1ENR_TIM2EN
            | RCC_APB1ENR_TIM4EN
            | RCC_APB1ENR_I2C1EN
            ;
    RCC->AHBENR = 0
            | RCC_AHBENR_FLITFEN
            | RCC_AHBENR_SRAMEN
            | RCC_AHBENR_DMA1EN;

    GPIOA->CRL =
            GPIO_CRL_CNF0_0
            | GPIO_CRL_CNF1_0
            | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1
            | GPIO_CRL_CNF3_0
            // noise level analogue in
            | 0
            | GPIO_CRL_MODE5_1
            | GPIO_CRL_CNF6_0
            | GPIO_CRL_CNF7_0;
    GPIOA->BSRR = GPIO_BSRR_BR5;

    GPIOA->CRH = 0
            // TIM1_CH1, DHT recv
            | GPIO_CRH_CNF8_0
            // USART1 TX
            | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1
            // USART1 RX
            | GPIO_CRH_CNF10_0
            | GPIO_CRH_CNF11_0
            | GPIO_CRH_CNF12_0
            | GPIO_CRH_CNF13_0
            | GPIO_CRH_CNF14_0
            | GPIO_CRH_CNF15_0;

    GPIOB->CRL =
            GPIO_CRL_CNF0_0
            | GPIO_CRL_CNF1_0
            | GPIO_CRL_CNF2_0
            | GPIO_CRL_CNF3_0
            | GPIO_CRL_CNF4_0
            // DHT send
            | GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_0
            | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0
            | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0
            ;

    GPIOB->CRH =
            GPIO_CRH_CNF8_0
            | GPIO_CRH_CNF9_0
            | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1
            | GPIO_CRH_CNF11_0
            | GPIO_CRH_CNF12_0
            | GPIO_CRH_CNF13_0
            | GPIO_CRH_MODE14_1
            | GPIO_CRH_MODE15_1;

    GPIOD->CRL =
            GPIO_CRL_CNF0_0
            | GPIO_CRL_CNF1_0
            | GPIO_CRL_CNF2_0
            | GPIO_CRL_CNF3_0
            | GPIO_CRL_CNF4_0
            | GPIO_CRL_CNF5_0
            | GPIO_CRL_CNF6_0
            | GPIO_CRL_CNF7_0;

    I2C1->CR1 = 0;

    usart1.init(115200);
    usart2.init(115200);
    usart3.init(115200);
    ls_freq_init();
    imu_timed_init();
    stm32_clock::init();
    // stm32_rtc::init();
    i2c_init();

    i2c_enable();
    i2c_workaround_reset();

    static const uint8_t config_20[] = {
        // at 0x20
        // data rate = 200 Hz  |  block data update  |  enable all three axis
        0x70 | 0x08 | 0x07,
        // anti-alias filter = 194 Hz
        0x40,
        //0x00,
    };
    /*static const uint8_t config_24[] = {
        // at 0x24
        // temperature sensor enabled  |  high resolution  |  data rate = 3.125 Hz
        0x80 | 0x60,
        // full scale = ±2 gauss
        0x00,
        0x00,
    };*/

    /*static const uint8_t config_20[] = {
        0x67,
        0x00
    };*/

    static const uint8_t config_24[] = {
        0xf4,
        0x00,
        0x00
    };

    i2c_smbus_write(0x1d, 0x20, 2, &config_20[0]);
    i2c_smbus_write(0x1d, 0x24, 3, &config_24[0]);

    /*ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_TSVREFE;
    ADC1->SMPR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->SQR1 = 0
            ;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0
            | ADC_SQR3_SQ1_2
            | ADC_SQR3_SQ2_4
            ;

    ADC1->CR2 = ADC_CR2_ADON;

    // give it a moment to settle
    for (uint32_t i = 0; i < 10000; ++i) {
        __asm__ volatile("nop");
    }

    ADC1->CR2 |= ADC_CR2_CAL;

    // wait for calibration to finish
    while (ADC1->CR2 & ADC_CR2_CAL);

    ADC1->CR1 |= ADC_CR1_SCAN;

    while (1) {
        ADC1->CR2 |= ADC_CR2_ADON;
        while (!(ADC1->SR & ADC_SR_EOC));

        char buf[5];
        buf[4] = '\n';
        uint16_to_hex(ADC1->DR, buf);
        usart2.send((const uint8_t*)buf, 5);

        uint16_to_hex(ADC1->DR, buf);
        usart2.send((const uint8_t*)buf, 5);

        for (uint32_t i = 0; i < 1000000; ++i) {
            __asm__ volatile("nop");
        }
    }*/


    usart1.enable();
    usart2.enable();
    usart3.enable();
    ls_freq_enable();
    imu_timed_enable();
    stm32_clock::enable();
    // stm32_rtc::enable();

    scheduler.add_task(&commtx);
    scheduler.add_task(&commrx);
    scheduler.add_task(&blink);
    scheduler.add_task(&stream_ax, IMU_SOURCE_ACCELEROMETER, 0);
    scheduler.add_task(&stream_ay, IMU_SOURCE_ACCELEROMETER, 1);
    scheduler.add_task(&stream_az, IMU_SOURCE_ACCELEROMETER, 2);
    scheduler.add_task(&stream_mx, IMU_SOURCE_MAGNETOMETER, 0);
    scheduler.add_task(&stream_my, IMU_SOURCE_MAGNETOMETER, 1);
    scheduler.add_task(&stream_mz, IMU_SOURCE_MAGNETOMETER, 2);
    scheduler.add_task(&sample_lightsensor);
    scheduler.add_task(&misc);

    scheduler.run();

    while (1) {}

    return 0;
}


#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
    usart2.send((uint8_t*)&ch, 1);
    return ch;
}

int puts(const char *s) {
    usart2.sendstr(s);
    return 0;
}
