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
#include "crc8.h"
#include "adc.h"
#include "bme280.h"

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
    std::array<uint8_t, MAX_FRAME_PAYLOAD_SIZE> m_packet_buffer;
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


class SampleOneWire: public Coroutine {
public:
    SampleOneWire(OnewireCore &core, CommInterfaceTX &tx):
        m_core(core),
        m_tx(tx),
        m_crc(0x31)
    {

    }

private:
    OnewireCore &m_core;
    CommInterfaceTX &m_tx;
    onewire_addr_t m_addr;
    uint8_t m_byte;
    std::array<uint8_t, 9> m_scratchpad;
    CRC8 m_crc;
    stm32_clock::time_point m_last_wakeup;

    CommInterfaceTX::buffer_t::buffer_handle_t m_handle;
    sbx_msg_t *m_buf;

    uint8_t m_sample_idx;
    stm32_clock::time_point m_timestamp;

private:
    void emit_buffer()
    {
        m_buf->type = sbx_msg_type::SENSOR_DS18B20;
        m_buf->payload.ds18b20.timestamp = m_timestamp.raw();
        m_tx.buffer().set_ready(
                    m_handle,
                    sizeof(sbx_msg_type) + sizeof(sbx_uptime_t) +
                    sizeof(sbx_msg_ds18b20_sample_t)*m_sample_idx);
        m_handle = CommInterfaceTX::buffer_t::INVALID_BUFFER;
        m_sample_idx = 0;
    }

public:
    void operator()() {
        Coroutine::operator()();
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            m_last_wakeup = now;

            await_call(m_core.detect_devices);
            if (m_core.detect_devices.status() != ONEWIRE_PRESENCE) {
                // no devices? -> sleep 10s
                m_byte = 0xff;
                await(usart2.send_c(&m_byte, 1));
                await(sleep_c(10000, m_last_wakeup));
                continue;
            }

            await_call(m_core.write_bytes, &ONEWIRE_SKIP_ROM, 1);
            await_call(m_core.write_bytes, &ONEWIRE_CONVERT_T, 1);
            m_timestamp = sched_clock::now();
            await(sleep_c(1500, now));

            memset(&m_addr, 0, sizeof(onewire_addr_t));
            m_handle = CommInterfaceTX::buffer_t::INVALID_BUFFER;

            do {
                await_call(m_core.find_next, m_addr);
                if (m_core.find_next.status() == ONEWIRE_PRESENCE) {
                    if (m_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER) {
                        do {
                            await(m_tx.buffer().any_buffer_free());
                            m_handle = m_tx.buffer().allocate(
                                        *(uint8_t**)&m_buf,
                                        sizeof(sbx_msg_type) + sizeof(sbx_msg_ds18b20_t)
                                        );
                        } while (m_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER);
                        m_sample_idx = 0;
                    }

                    memset(&m_scratchpad[0], 0, 9);
                    await_call(m_core.write_bytes, &ONEWIRE_READ_SCRATCHPAD, 1);
                    await_call(m_core.read_bytes, &m_scratchpad[0], 9);
                    m_crc.reset();
                    m_byte = m_crc.feed(&m_scratchpad[0], 9);
                    if (m_byte != 0x00) {
                        // CRC fail, skip sending the packet
                        continue;
                    }
                    memcpy(&m_buf->payload.ds18b20.samples[m_sample_idx].id[0], &m_addr[0],
                           ONEWIRE_ADDR_LEN);
                    memcpy(&m_buf->payload.ds18b20.samples[m_sample_idx].raw_value,
                           &m_scratchpad[0], 2);
                    m_sample_idx += 1;

                    if (m_sample_idx == SBX_MAX_DS18B20_SAMPLES) {
                        emit_buffer();
                    }
                }
            } while (m_core.find_next.status() == ONEWIRE_PRESENCE);

            if (m_sample_idx > 0) {
                emit_buffer();
            } else if (m_handle != CommInterfaceTX::buffer_t::INVALID_BUFFER) {
                m_tx.buffer().release(m_handle);
                m_handle = CommInterfaceTX::buffer_t::INVALID_BUFFER;
            }

            await(sleep_c(10000, m_last_wakeup));
        }
        COROUTINE_END;
    }

};


class SampleADC: public Coroutine
{
public:
    explicit SampleADC(CommInterfaceTX &tx):
        m_tx(tx)
    {

    }

private:
    CommInterfaceTX &m_tx;
    CommInterfaceTX::buffer_t::buffer_handle_t m_handle;
    sbx_msg_t *m_buf;
    sched_clock::time_point m_last_wakeup;
    uint8_t m_sample;
    uint16_t m_value_buf;

public:
    void operator()()
    {

    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        m_last_wakeup = now;
        while (1) {
            do {
                await(m_tx.buffer().any_buffer_free());
                m_handle = m_tx.buffer().allocate(
                            *(uint8_t**)&m_buf,
                            sizeof(sbx_msg_type) + sizeof(sbx_msg_noise_t)
                            );
            } while (m_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER);

            for (m_sample = 0; m_sample < SBX_NOISE_SAMPLES; m_sample++) {
                await(sleep_c(1000, m_last_wakeup));
                m_last_wakeup = now;
                m_buf->payload.noise.samples[m_sample].timestamp = sched_clock::now_raw();
                await(adc_sample(4, m_value_buf));
                m_buf->payload.noise.samples[m_sample].value = m_value_buf;
            }

            m_buf->type = sbx_msg_type::SENSOR_NOISE;

            m_tx.buffer().set_ready(m_handle);
        }
        COROUTINE_END;
    }

};


class SampleBME280: public Coroutine
{
public:
    struct Metrics {
        Metrics():
            configure_status(0xff),
            timeouts(0)
        {

        }

        uint8_t configure_status;
        uint16_t timeouts;
    };

public:
    explicit SampleBME280(CommInterfaceTX &tx, I2C &bus):
        m_tx(tx),
        m_bus(bus),
        m_detect_and_configure(bus)
    {

    }

private:
    CommInterfaceTX &m_tx;
    CommInterfaceTX::buffer_t::buffer_handle_t m_handle;
    I2C &m_bus;
    sbx_msg_t *m_buf;
    sched_clock::time_point m_last_wakeup;
    WaitFor m_wait_for;
    bool m_timed_out;
    bool m_was_present;
    uint8_t m_detect_status;

    Metrics m_metrics;
    BME280DetectAndConfigure m_detect_and_configure;

public:
    inline const Metrics &metrics() const
    {
        return m_metrics;
    }

    void operator()()
    {
        m_was_present = false;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        while (1) {
            m_last_wakeup = now;

            await_call(m_detect_and_configure, BME280_DEVICE_ADDRESS, m_detect_status, m_was_present);
            m_metrics.configure_status = m_detect_status;
            if (m_detect_status != BME280_OK) {
                // do not loop tightly if BME280 not found
                m_was_present = false;
                await(sleep_c(5000, m_last_wakeup));
                continue;
            }
            m_was_present = true;

            do {
                await(m_tx.buffer().any_buffer_free());
                m_handle = m_tx.buffer().allocate(
                            *(uint8_t**)&m_buf,
                            sizeof(sbx_msg_type) + sizeof(sbx_msg_bme280_t)
                            );
            } while (m_handle == CommInterfaceTX::buffer_t::INVALID_BUFFER);

            m_buf->type = sbx_msg_type::SENSOR_BME280;

            await_call(m_wait_for, i2c2.smbus_readc(BME280_DEVICE_ADDRESS, 0x88, SBX_BME280_DIG88_SIZE, &m_buf->payload.bme280.dig88[0]), 10, m_timed_out);
            if (m_timed_out) {
                m_metrics.timeouts += 1;
                m_tx.buffer().release(m_handle);
                await(sleep_c(5000, m_last_wakeup));
                continue;
            }

            await_call(m_wait_for, i2c2.smbus_readc(BME280_DEVICE_ADDRESS, 0xe1, SBX_BME280_DIGE1_SIZE, &m_buf->payload.bme280.dige1[0]), 10, m_timed_out);
            if (m_timed_out) {
                m_metrics.timeouts += 1;
                m_tx.buffer().release(m_handle);
                await(sleep_c(5000, m_last_wakeup));
                continue;
            }

            m_buf->payload.bme280.timestamp = sched_clock::now_raw();

            await_call(m_wait_for, i2c2.smbus_readc(BME280_DEVICE_ADDRESS, BME280_DATA_START, SBX_BME280_READOUT_SIZE, &m_buf->payload.bme280.readout[0]), 10, m_timed_out);
            if (m_timed_out) {
                m_metrics.timeouts += 1;
                m_tx.buffer().release(m_handle);
                await(sleep_c(5000, m_last_wakeup));
                continue;
            }

            m_tx.buffer().set_ready(m_handle);

            await(sleep_c(5000, m_last_wakeup));
        }
        COROUTINE_END;
    }

};


class MiscTask: public Coroutine {
public:
    MiscTask(CommInterfaceTX &tx, SampleBME280 &sample_bme280):
        m_tx(tx),
        m_sample_bme280(sample_bme280)
    {

    }

private:
    CommInterfaceTX &m_tx;
    sched_clock::time_point m_last_wakeup;
    CommInterfaceTX::buffer_t::buffer_handle_t m_handle;
    sbx_msg_t *m_buf;
    SampleBME280 &m_sample_bme280;

    uint16_t m_tmp_seq;
    uint16_t m_tmp_timestamp;

    inline void copy_i2c_metrics(const I2C &i2c, unsigned i)
    {
        auto &src_metrics = i2c.metrics();
        m_buf->payload.status.i2c_metrics[i].transaction_overruns = src_metrics.transaction_overruns;
    }

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
            m_buf->payload.status.rtc = 0xdeadbeef;
            m_buf->payload.status.protocol_version = 0x01;
            m_buf->payload.status.status_version = 0x03;
            imu_timed_get_state(
                        IMU_SOURCE_ACCELEROMETER,
                        m_tmp_seq,
                        m_tmp_timestamp);
            m_buf->payload.status.imu.stream_state[0].sequence_number = m_tmp_seq;
            m_buf->payload.status.imu.stream_state[0].timestamp = m_tmp_timestamp;
            imu_timed_get_state(
                        IMU_SOURCE_MAGNETOMETER,
                        m_tmp_seq,
                        m_tmp_timestamp);
            m_buf->payload.status.imu.stream_state[1].sequence_number = m_tmp_seq;
            m_buf->payload.status.imu.stream_state[1].timestamp = m_tmp_timestamp;
            m_buf->payload.status.imu.stream_state[0].period = 5;
            m_buf->payload.status.imu.stream_state[1].period = 64*5;
            copy_i2c_metrics(i2c1, 0);
            copy_i2c_metrics(i2c2, 1);
            {
                auto &metrics = m_sample_bme280.metrics();
                m_buf->payload.status.bme280_metrics.configure_status = metrics.configure_status;
                m_buf->payload.status.bme280_metrics.timeouts = metrics.timeouts;
            }

            m_buf->payload.status.uptime = sched_clock::now_raw();
            m_tx.buffer().set_ready(m_handle);

            // "sample" the RTC in prime intervals to get more accurate estimate
            // on where the second boundary is
            await(sleep_c(487, m_last_wakeup));
        }
        COROUTINE_END;
    }
};


static BlinkLED blink;
static OnewireCore onewire(usart1);
static CommInterfaceTX commtx(usart3);
// static CommInterfaceRX commrx(usart3);
static IMUSensorStream stream_ax(commtx);
static IMUSensorStream stream_ay(commtx);
static IMUSensorStream stream_az(commtx);
static IMUSensorStream stream_mx(commtx);
static IMUSensorStream stream_my(commtx);
static IMUSensorStream stream_mz(commtx);
static SampleLightsensor sample_lightsensor(commtx);
static SampleOneWire sample_onewire(onewire, commtx);
static SampleADC sample_adc(commtx);
static SampleBME280 sample_bme280(commtx, i2c2);
static MiscTask misc(commtx, sample_bme280);
static Scheduler<13> scheduler;


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

    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_I2C2RST | RCC_APB1RSTR_USART2RST | RCC_APB1RSTR_USART3RST;
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_IOPCRST | RCC_APB2RSTR_IOPDRST | RCC_APB2RSTR_IOPERST | RCC_APB2RSTR_TIM1RST | RCC_APB2RSTR_USART1RST | RCC_APB2RSTR_ADC1RST;

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
            | RCC_APB1ENR_I2C2EN
            ;
    RCC->AHBENR = 0
            | RCC_AHBENR_FLITFEN
            | RCC_AHBENR_SRAMEN
            | RCC_AHBENR_DMA1EN;

    // give it a moment to reset
    for (uint32_t i = 0; i < 1000; ++i) {
        __asm__ volatile("nop");
    }

    RCC->APB1RSTR = 0;
    RCC->APB2RSTR = 0;

    // give it a moment to power the peripherials up
    for (uint32_t i = 0; i < 1000; ++i) {
        __asm__ volatile("nop");
    }

    DMA1->IFCR = 0xfffffff;  // clear all the interrupt flags
    // manually reset all the DMAs
    DMA1_Channel1->CCR = 0;
    DMA1_Channel2->CCR = 0;
    DMA1_Channel3->CCR = 0;
    DMA1_Channel4->CCR = 0;
    DMA1_Channel5->CCR = 0;
    DMA1_Channel6->CCR = 0;
    DMA1_Channel7->CCR = 0;

    // and another moment to complete the DMA reset
    for (uint32_t i = 0; i < 1000; ++i) {
        __asm__ volatile("nop");
    }


    // Remap USART3 RX/TX to other pins
    AFIO->MAPR = 0
            | AFIO_MAPR_USART3_REMAP_PARTIALREMAP;

    GPIOA->CRL =
            GPIO_CRL_MODE0_1
            | GPIO_CRL_MODE1_1
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
            // USART1 TX (in open-drain mode)
            | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF9_0
            // USART1 RX
            | GPIO_CRH_CNF10_0
            | GPIO_CRH_CNF11_0
            | GPIO_CRH_CNF12_0
            | GPIO_CRH_CNF13_0
            | GPIO_CRH_CNF14_0
            | GPIO_CRH_CNF15_0;

    GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7 |
                                 GPIO_CRL_CNF6 | GPIO_CRL_CNF7))
        ;

    GPIOB->CRL =
            GPIO_CRL_CNF0_0
            | GPIO_CRL_CNF1_0
            | GPIO_CRL_CNF2_0
            | GPIO_CRL_CNF3_0
            | GPIO_CRL_CNF4_0
            // DHT send
            | GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0 | GPIO_CRL_CNF5_0
            // I2C1 SCL
            | GPIO_CRL_MODE6_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_0
            // I2C1 SDA
            | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_0
            ;

    GPIOB->CRH =
            GPIO_CRH_CNF8_0
            | GPIO_CRH_CNF9_0
            // USART3 TX (original position, but we remapped it to PC10)
            // | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1
            // USART3 RX (original position, but we remapped it to PC11)
            // | GPIO_CRH_CNF11_0
            // I2C2 SCL
            | GPIO_CRH_MODE10_0 | GPIO_CRH_CNF10_1 | GPIO_CRH_CNF10_0
            // I2C2 SDA
            | GPIO_CRH_MODE11_0 | GPIO_CRH_CNF11_1 | GPIO_CRH_CNF11_0
            | GPIO_CRH_CNF12_0
            | GPIO_CRH_CNF13_0
            | GPIO_CRH_CNF14_0
            | GPIO_CRH_CNF15_0;

    GPIOC->CRH = 0
            // USART3 TX (remapped!)
            | GPIO_CRH_MODE10_1 | GPIO_CRH_CNF10_1
            // USART3 RX (remapped!)
            | GPIO_CRH_CNF11_0;

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
    I2C2->CR1 = 0;

    usart1.init(115200);
    usart2.init(115200);
    usart3.init(115200);
    ls_freq_init();
    imu_timed_init();
    stm32_clock::init();
    // stm32_rtc::init();
    adc_init();
    i2c1.init();
    i2c2.init();

    i2c1.enable();
    i2c2.enable();

    i2c1_workaround_reset();

    imu_timed_configure(i2c1);

    usart1.enable();
    usart2.enable();
    usart3.enable();
    adc_enable();
    ls_freq_enable();
    imu_timed_enable();
    stm32_clock::enable();
    // stm32_rtc::enable();

    // puts("bootup complete!\n");

    scheduler.add_task(&commtx);
    // scheduler.add_task(&commrx);
    scheduler.add_task(&blink);
    scheduler.add_task(&stream_ax, IMU_SOURCE_ACCELEROMETER, 0);
    scheduler.add_task(&stream_ay, IMU_SOURCE_ACCELEROMETER, 1);
    scheduler.add_task(&stream_az, IMU_SOURCE_ACCELEROMETER, 2);
    scheduler.add_task(&stream_mx, IMU_SOURCE_MAGNETOMETER, 0);
    scheduler.add_task(&stream_my, IMU_SOURCE_MAGNETOMETER, 1);
    scheduler.add_task(&stream_mz, IMU_SOURCE_MAGNETOMETER, 2);
    scheduler.add_task(&sample_lightsensor);
    scheduler.add_task(&misc);
    scheduler.add_task(&sample_onewire);
    scheduler.add_task(&sample_adc);
    scheduler.add_task(&sample_bme280);

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
