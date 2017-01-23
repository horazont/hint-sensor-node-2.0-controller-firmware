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
#include "comm_xbee.h"
#include "usart.h"

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


using CommInterface = CommXBEE;


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
            await(sleepc(500, now));
            GPIOA->BSRR = GPIO_BSRR_BS5;
            await(sleepc(500, now));
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        COROUTINE_END;
    }
};


class PackBuffer: public Coroutine
{
public:
    PackBuffer():
        Coroutine(),
        m_need_reset(true),
        m_hangover_len(0)
    {
    }

private:
    bool m_need_reset;

    std::array<uint16_t, IMU_BUFFER_LENGTH> m_hangover_buffer;
    uint_fast8_t m_hangover_len;


private:
    const imu_buffer_t *m_src_buffer;
    uint_fast8_t m_src_offset;

    sbx_msg_t *m_msg_out;
    uint8_t *m_packet_size_out;
    uint16_t m_packet_samples;

    uint16_t m_average;

    std::array<uint8_t, SENSOR_STREAM::MAX_BITMAP_SIZE> m_bitmap_buffer;
    std::array<uint8_t, MAX_XBEE_PAYLOAD_SIZE> m_packet_buffer;
    uint8_t m_bitmap_byte;
    uint8_t m_bitmap_bit;

    uint8_t m_payload_size;
    uint8_t m_samples_used;
    uint8_t m_packet_pos;

    uint8_t m_i;

    inline void reset()
    {
        memset(&m_bitmap_buffer[0], 0, m_bitmap_buffer.size());
        m_bitmap_bit = 7;
        m_bitmap_byte = 0;
        m_payload_size = 0;
        m_samples_used = 0;
        m_packet_pos = 0;
        m_packet_samples = 0;
    }

    inline bool pack_sample(uint16_t sample)
    {
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

        m_samples_used += 1;
        m_packet_samples += 1;

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
        m_msg_out->type = static_cast<sbx_msg_type>(
                    static_cast<uint8_t>(sbx_msg_type::SENSOR_STREAM_ACCEL_X) +
                    m_src_offset);
        m_msg_out->payload.sensor_stream.seq = m_src_buffer->seq;
        m_msg_out->payload.sensor_stream.average = m_average;
        memcpy(&m_msg_out->payload.sensor_stream.data[0],
                &m_bitmap_buffer[0],
                bitmap_len);
        memcpy(&m_msg_out->payload.sensor_stream.data[bitmap_len],
               &m_packet_buffer[0],
                m_packet_pos);

        *m_packet_size_out =
                bitmap_len + m_packet_pos +
                sizeof(sbx_msg_type) +
                sizeof(sbx_msg_sensor_stream_t);

        m_need_reset = true;
    }

public:
    void operator()(const imu_buffer_t &buffer,
                    const uint_fast8_t src_offset,
                    uint8_t &packet_size_out,
                    sbx_msg_t &msg_out)
    {
        Coroutine::operator()();
        m_src_buffer = &buffer;
        m_src_offset = src_offset;
        m_packet_size_out = &packet_size_out;
        m_msg_out = &msg_out;
    }

    inline uint8_t samples_in_msg() const
    {
        return m_packet_samples;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        if (m_need_reset) {
            reset();
            m_need_reset = false;

            // we have a buffer, now what?
            // we need to pack the data, and we need to keep data which was
            // unpackable
            m_average = m_src_buffer->samples[0].accel_compass[m_src_offset];

            if (m_hangover_len) {
                for (m_i = 0; m_i < m_hangover_len; ++m_i) {
                    // hangover buffer is too small to cause a packet to be emitted
                    pack_sample(m_hangover_buffer[m_i]);
                }
                m_hangover_len = 0;
            }
        }

        m_samples_used = 0;
        for (m_i = 0; m_i < m_src_buffer->samples.size(); ++m_i)
        {
            if (!pack_sample(m_src_buffer->samples[m_i].accel_compass[m_src_offset])) {
                break;
            }
        }

        if (m_samples_used == m_src_buffer->samples.size() &&
                m_payload_size < SENSOR_STREAM::MAX_ENCODED_SAMPLE_BYTES-3)
        {
            // there is enough room left for more data
            m_packet_size_out = 0;
            COROUTINE_RETURN;
        }

        m_hangover_len = 0;
        for (m_i = m_samples_used; m_i < m_src_buffer->samples.size(); ++m_i) {
            m_hangover_buffer[m_i-m_samples_used] = m_src_buffer->samples[m_i].accel_compass[m_src_offset];
            m_hangover_len += 1;
        }

        yield;
        finish_packet();
        COROUTINE_END;
    }
};


class IMUStream: public Coroutine
{
public:
    explicit IMUStream(CommInterface &comm):
        m_comm(comm)
    {

    }

private:
    CommInterface &m_comm;

private:
    union {
        uint8_t r8[8];
        uint16_t r16[6];
    } m_reg;

    struct channel_stats_t
    {
        uint16_t t0;
        uint16_t bytes_sent;
        uint16_t samples_sent;
        uint16_t packets_sent;
    };

    struct channel_buffer_t
    {
        CommInterface::buffer_t::buffer_handle_t m_msg_handle;
        uint8_t m_packet_size;
        uint8_t *m_msg;
        uint16_t m_len;
    };

    char m_buf[100];
    const imu_buffer_t *m_buffer;
    PackBuffer m_packers[6];
    channel_stats_t m_stats[6];
    channel_buffer_t m_buffers[6];
    uint8_t m_i;
    uint16_t m_k;

private:
    uint16_t format_stats(const uint8_t channel, const uint16_t t1)
    {
        channel_stats_t &stats = m_stats[channel];
        const uint16_t bits_per_sample = ((uint32_t)stats.bytes_sent * 8) / (stats.samples_sent);

        const uint16_t dt = t1 - stats.t0;

        const uint16_t bytes_per_second1 = (stats.bytes_sent / (dt/1000));
        const uint16_t bytes_per_second2 = ((uint32_t)stats.bytes_sent * 100) / (stats.samples_sent);
        char *dest = &m_buf[0];
        uint8_to_hex(channel, dest);
        dest += 2;
        *dest++ = ':';
        *dest++ = ' ';
        uint16_to_hex(stats.packets_sent, dest);
        dest += 4;
        *dest++ = ' ';
        uint16_to_hex(bits_per_sample, dest);
        dest += 4;
        *dest++ = ' ';
        uint16_to_hex(bytes_per_second1, dest);
        dest += 4;
        *dest++ = ' ';
        uint16_to_hex(bytes_per_second2, dest);
        dest += 4;
        *dest++ = '\n';

        stats.t0 = t1;
        stats.packets_sent = 0;
        stats.bytes_sent = 0;
        stats.samples_sent = 0;

        return dest - m_buf;
    }

public:
    void operator()()
    {
        Coroutine::operator()();
        m_reg.r16[0] = 0xdead;
        m_reg.r16[1] = 0xbeef;
        m_reg.r16[2] = 0xdead;
        m_reg.r16[3] = 0xbeef;
        m_reg.r16[4] = 0xdead;
        m_reg.r16[5] = 0xbeef;
        m_buf[2] = ' ';
        m_buf[3] = 0;
        m_buf[4] = ' ';
        m_buf[5] = 0;

        for (unsigned int i = 0; i < 6; ++i) {
            m_stats[0].bytes_sent = 0;
            m_stats[0].samples_sent = 0;
            m_stats[0].packets_sent = 0;
        }
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        static const uint8_t config_20[] = {
            // 0x20
            0x67,
            0x00,
        };
        static const uint8_t config_24[] = {
            // 0x24
            0xf4,
            0x00,
            0x00,
        };

        /*static const uint8_t delimiter1[2] = {0xde, 0xad};
        static const uint8_t delimiter2[2] = {0xbe, 0xef};*/

        await(i2c_smbus_writec(0x1d, 0x20, 2, &config_20[0]));
        await(i2c_smbus_writec(0x1d, 0x24, 3, &config_24[0]));
        await(i2c_smbus_readc(0x1d, 0x1f, 8, &m_reg.r8[0]));
        imu_timed_init();
        imu_timed_enable();

        {
            uint16_t t0 = sched_clock::now_raw();
            for (unsigned int i = 0; i < 6; ++i) {
                m_stats[i].t0 = t0;
                m_buffers[i].m_msg_handle = CommInterface::buffer_t::INVALID_BUFFER;
            }
        }

        m_k = 0;
        await(usart3.tx_ready());
        /*await(usart3.send_c(delimiter1, 2));
        await(usart3.send_c(delimiter2, 2));*/
        while (1) {
            for (m_i = 0; m_i < 6; ++m_i) {
                while (m_buffers[m_i].m_msg_handle == CommInterface::buffer_t::INVALID_BUFFER) {
                    await(m_comm.output_buffer().any_buffer_free());
                    m_buffers[m_i].m_msg_handle = m_comm.output_buffer().allocate(
                                m_buffers[m_i].m_msg,
                                CommInterface::buffer_t::BUFFER_SIZE,
                                PRIO_NO_RETRIES);
                }
            }
            await(imu_timed_full_buffer(m_buffer));
            // await(usart3.send_c((uint8_t*)m_buffer, sizeof(imu_buffer_t)));
            for (m_i = 0; m_i < 6; ++m_i) {
                await_call(m_packers[m_i], *m_buffer, m_i, m_buffers[m_i].m_packet_size,
                           *(sbx_msg_t*)m_buffers[m_i].m_msg);
                if (m_buffers[m_i].m_packet_size) {
                    // packet finished
                    //await(usart3.tx_ready());
                    //await(usart3.send_c((uint8_t*)m_msg, m_packet_size));
                    /*m_stats[m_i].packets_sent += 1;
                    m_stats[m_i].bytes_sent += m_packet_size;
                    m_stats[m_i].samples_sent += m_packers[m_i].samples_in_msg();*/
                    m_comm.output_buffer().set_ready(
                                m_buffers[m_i].m_msg_handle,
                                m_buffers[m_i].m_packet_size);
                    m_buffers[m_i].m_msg_handle = CommInterface::buffer_t::INVALID_BUFFER;
                }
            }

            /*m_k++;
            if (m_k == 400) {
                for (m_i = 0; m_i < 6; ++m_i) {
                    m_len = format_stats(m_i, sched_clock::now_raw());
                    await(usart3.send_c((const uint8_t*)m_buf, m_len));
                }
                m_k = 0;
            }*/
        }

        COROUTINE_END;
    }
};

void delay() {
    for (uint32_t i = 0; i < 800000; ++i) {
        __asm__ volatile("nop");
    }
}


static BlinkLED blink;
//static I2CSensor sensor;
static CommInterface comm(usart3);
static IMUStream stream(comm);
static Scheduler<3> scheduler;


int main() {
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_I2C1RST | RCC_APB1RSTR_USART2RST | RCC_APB1RSTR_USART3RST;
    RCC->APB2RSTR |= RCC_APB2RSTR_IOPARST | RCC_APB2RSTR_IOPBRST | RCC_APB2RSTR_IOPCRST | RCC_APB2RSTR_IOPDRST | RCC_APB2RSTR_IOPERST;

    delay();

    RCC->APB1RSTR = 0;
    RCC->APB2RSTR = 0;

    GPIOA->CRL =
            GPIO_CRL_CNF0_0
            | GPIO_CRL_CNF1_0
            | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1
            | GPIO_CRL_CNF3_0
            | GPIO_CRL_CNF4_0
            | GPIO_CRL_MODE5_1
            | GPIO_CRL_CNF6_0
            | GPIO_CRL_CNF7_0;
    GPIOA->BSRR = GPIO_BSRR_BR5;

    GPIOB->CRL =
            GPIO_CRL_CNF0_0
            | GPIO_CRL_CNF1_0
            | GPIO_CRL_CNF2_0
            | GPIO_CRL_CNF3_0
            | GPIO_CRL_CNF4_0
            | GPIO_CRL_CNF5_0
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
            | GPIO_CRH_CNF14_0
            | GPIO_CRH_CNF15_0;

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

    RCC->APB2ENR |= 0
            | RCC_APB2ENR_IOPAEN
            | RCC_APB2ENR_IOPBEN
            | RCC_APB2ENR_IOPCEN
            | RCC_APB2ENR_IOPDEN
            | RCC_APB2ENR_IOPEEN
            | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= 0
            | RCC_APB1ENR_TIM3EN
            | RCC_APB1ENR_USART2EN
            | RCC_APB1ENR_USART3EN
            | RCC_APB1ENR_TIM2EN
            | RCC_APB1ENR_TIM4EN
            | RCC_APB1ENR_I2C1EN
            ;
    RCC->AHBENR |= 0
            | RCC_AHBENR_DMA1EN;

    usart2.init(115200, false);
    usart2.enable();

    usart3.init(115200, false);
    usart3.enable();

    i2c_init();
    i2c_enable();
    i2c_workaround_reset();

    stm32_clock::init();
    stm32_clock::enable();

    scheduler.add_task(&blink);
    scheduler.add_task(&stream);
    scheduler.add_task(&comm);

    scheduler.run();

    while (1) {}

    return 0;
}


#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
    /*while (!(USART2->SR & USART_FLAG_TC));
    USART2->DR = ch;
    while (!(USART2->SR & USART_FLAG_TC));*/
    usart2.send((uint8_t*)&ch, 1);
    return ch;
}

int puts(const char *s) {
    while (*s != 0) {
        __io_putchar(*s);
        s++;
    }
    return 0;
}
