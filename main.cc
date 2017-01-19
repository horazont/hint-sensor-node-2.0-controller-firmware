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
        m_previous_average_known(false),
        m_need_reset(true),
        m_average_accum(0),
        m_average_accum_ctr(0),
        m_seq(0),
        m_hangover_len(0)
    {
    }

private:
    bool m_previous_average_known;
    bool m_need_reset;
    uint16_t m_previous_average;
    int32_t m_average_accum;
    uint_fast8_t m_average_accum_ctr;
    uint8_t m_seq;

    std::array<uint16_t, IMU_BUFFER_LENGTH> m_hangover_buffer;
    uint_fast8_t m_hangover_len;


private:
    const imu_buffer_t *m_src_buffer;
    uint_fast8_t m_src_offset;

    const sbx_msg_t **m_msg_out;
    uint8_t *m_packet_size_out;

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
        memset(&m_packet_buffer[0], 0, m_packet_buffer.size());
        m_bitmap_bit = 7;
        m_bitmap_byte = 0;
        m_payload_size = 0;
        m_samples_used = 0;
        m_packet_pos = 0;
    }

    inline bool pack_sample(uint16_t sample)
    {
        m_average_accum += (int16_t)sample;
        m_average_accum_ctr += 1;
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
        m_previous_average_known = m_average_accum_ctr > 0;
        if (m_previous_average_known) {
            m_previous_average = (uint16_t)(m_average_accum / m_average_accum_ctr);
        }
        m_average_accum = 0;
        m_average_accum_ctr = 0;

        const uint8_t bitmap_len = (m_bitmap_bit == 7 ? m_bitmap_byte : m_bitmap_byte+1);
        // const uint8_t total_len = bitmap_len + m_packet_pos + 2;
        sbx_msg_t &msg = *reinterpret_cast<sbx_msg_t*>(&m_packet_buffer.front());
        memmove(&msg.payload.sensor_stream.data[bitmap_len],
                &m_packet_buffer[0],
                m_packet_pos);
        memcpy(&msg.payload.sensor_stream.data[0],
                &m_bitmap_buffer[0],
                bitmap_len);
        msg.type = static_cast<sbx_msg_type>(
                    static_cast<uint8_t>(sbx_msg_type::SENSOR_STREAM_ACCEL_X) +
                    m_src_offset);
        msg.payload.sensor_stream.seq = m_seq++;
        msg.payload.sensor_stream.average = m_average;

        *m_packet_size_out =
                bitmap_len + m_packet_pos +
                sizeof(sbx_msg_type) +
                sizeof(sbx_msg_sensor_stream_t);
        *m_msg_out = &msg;

        m_need_reset = true;
    }

public:
    void operator()(const imu_buffer_t &buffer,
                    const uint_fast8_t src_offset,
                    uint8_t &packet_size_out,
                    const sbx_msg_t *&msg_out)
    {
        Coroutine::operator()();
        m_src_buffer = &buffer;
        m_src_offset = src_offset;
        m_packet_size_out = &packet_size_out;
        m_msg_out = &msg_out;
    }

    COROUTINE_DECL
    {
        COROUTINE_INIT;
        if (m_need_reset) {
            reset();
            m_need_reset = false;
        }

        // we have a buffer, now what?
        // we need to pack the data, and we need to keep data which was
        // unpackable
        m_average = (
                    m_previous_average_known ? m_previous_average : (*m_src_buffer)[0].accel_compass[m_src_offset]
                    );
//        {
//            char buf[10];
//            buf[8] = '\n';
//            buf[9] = 0;
//            uint32_to_hex(
//                        (m_previous_average << 16) | (m_previous_average_known << 8),
//                        buf);
//            puts(buf);
//        }

        /*if (m_hangover_len) {
//            puts("data in hangover buffer\n");
            for (m_i = 0; m_i < m_hangover_len; ++m_i) {
                // hangover buffer is too small to cause a packet to be emitted
                pack_sample(m_hangover_buffer[m_i]);
            }
            m_hangover_len = 0;
        }*/

        m_samples_used = 0;
        for (m_i = 0; m_i < (*m_src_buffer).size(); ++m_i)
        {
            if (!pack_sample((*m_src_buffer)[m_i].accel_compass[m_src_offset])) {
                break;
            }
        }
        if (m_samples_used == m_src_buffer->size() &&
                m_payload_size < SENSOR_STREAM::MAX_ENCODED_SAMPLE_BYTES-3)
        {
            // there is enough room left for more data
            *m_msg_out = nullptr;
//            puts("still room: ");
//            {
//                char buf[10];
//                buf[8] = '\n';
//                buf[9] = 0;
//                uint32_to_hex(
//                            m_samples_used | (m_payload_size << 8),
//                            buf);
//                puts(buf);
//            }
            COROUTINE_RETURN;
        }

        m_hangover_len = 0;
        for (m_i = m_samples_used; m_i < m_src_buffer->size(); ++m_i) {
            m_hangover_buffer[m_i] = (*m_src_buffer)[m_i].accel_compass[m_src_offset];
            m_hangover_len += 1;
        }

//        puts("full: ");
//        {
//            char buf[10];
//            buf[8] = '\n';
//            buf[9] = 0;
//            uint32_to_hex(
//                        m_samples_used | (m_payload_size << 8),
//                        buf);
//            puts(buf);
//        }

        yield;
        // TODO: fix dropping of unused samples!
        finish_packet();
        COROUTINE_END;
    }
};


class IMUStream: public Coroutine
{
private:
    union {
        uint8_t r8[8];
        uint16_t r16[6];
    } m_reg;
    char m_buf[6];
    const imu_buffer_t *m_buffer;
    PackBuffer m_accel_x_packer;
    uint8_t m_packet_size;
    const sbx_msg_t *m_msg;

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

        await(i2c_smbus_writec(0x1d, 0x20, 2, &config_20[0]));
        await(i2c_smbus_writec(0x1d, 0x24, 3, &config_24[0]));
        await(i2c_smbus_readc(0x1d, 0x1f, 8, &m_reg.r8[0]));
//        puts("recvd = ");
//        for (uint8_t i = 0; i < 8; ++i) {
//            uint8_to_hex(m_reg.r8[i], m_buf);
//            puts(m_buf);
//        }
//        puts("\n");
        imu_timed_init();
        imu_timed_enable();

        while (1) {
            await(imu_timed_full_buffer(m_buffer));
            await(usart2.tx_ready());
            await(usart2.send_c((uint8_t*)m_buffer, IMU_BUFFER_LENGTH*sizeof(imu_data_point_t)));
            await_call(m_accel_x_packer, *m_buffer, 0, m_packet_size, m_msg);
            if (m_msg) {
                // packet finished
                await(usart2.tx_ready());
                await(usart2.send_c((uint8_t*)m_msg, m_packet_size));
            }
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
static IMUStream stream;
static Scheduler<2> scheduler;


int main() {
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST | RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM4RST | RCC_APB1RSTR_I2C1RST;
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

    GPIOD->CRL =
        GPIO_CRL_CNF0_0
        | GPIO_CRL_CNF1_0
        | GPIO_CRL_CNF2_0
        | GPIO_CRL_CNF3_0
        | GPIO_CRL_CNF4_0
        | GPIO_CRL_CNF5_0
        | GPIO_CRL_CNF6_0
        | GPIO_CRL_CNF7_0;

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
        | RCC_APB1ENR_TIM2EN
        | RCC_APB1ENR_TIM4EN
        | RCC_APB1ENR_I2C1EN
        ;
    RCC->AHBENR |= 0
            | RCC_AHBENR_DMA1EN;

    usart2.init(115200, false);
    usart2.enable();

    i2c_init();
    i2c_enable();
    i2c_workaround_reset();

    stm32_clock::init();
    stm32_clock::enable();

    scheduler.add_task(&blink);
    scheduler.add_task(&stream);

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
