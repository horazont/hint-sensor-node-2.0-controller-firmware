#include "imu.h"

#include <stm32f10x.h>

#include <cstring>

#include "i2clib.h"
#include "notify.h"

struct imu_source_data_t
{
    imu_source_data_t():
        current_buffer(0),
        offset(0),
        sample_seq(0)
    {

    }

    std::array<notifier_t, 2> notifiers;
    std::array<imu_buffer_t, 2> buffers;
    volatile uint8_t current_buffer;
    uint8_t offset;
    uint16_t sample_seq;
    sched_clock::time_point last_sample;

    imu_buffer_t &buffer()
    {
        return buffers[current_buffer];
    }

    void swap_buffers()
    {
        uint_fast8_t curr_buffer = current_buffer;
        buffers[curr_buffer].first_sample = sample_seq;
        notifiers[curr_buffer].trigger();
        curr_buffer ^= 1;
        notifiers[curr_buffer].reset();
        current_buffer = curr_buffer;
        offset = 0;
    }

    ASYNC_CALLABLE wait_for_buffer(
            const imu_buffer_t *&full_buffer)
    {
        uint_fast8_t curr_buffer = current_buffer;
        full_buffer = &buffers[curr_buffer];
        return notifiers[curr_buffer].ready_c();
    }
};

static std::array<imu_source_data_t, 2> sources;
static uint8_t ctr;

static constexpr uint8_t IMU_DEVICE_ADDRESS = 0x1d;
static constexpr uint8_t IMU_ACCEL_ADDRESS = 0x28;
static constexpr uint8_t IMU_COMPASS_ADDRESS = 0x08;

static const uint8_t config_20[] = {
    // at 0x20
    // data rate = 200 Hz  |  block data update  |  enable all three axis
    0x70 | 0x08 | 0x07,
    // anti-alias filter = 194 Hz
    0x40,
    //0x00,
};

static const uint8_t config_24[] = {
    0xf4,
    0x00,
    0x00
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

void imu_timed_init()
{
    ctr = 0;

    TIM4->CR1 = 0;
    TIM4->CR2 = 0;
    TIM4->SMCR = 0;
    TIM4->DIER = TIM_DIER_UIE;

    // 2.5ms period
    TIM4->PSC = 999;
    // those fucking timer clocks are doubled! (but timers are on the slow
    // bus, so after doubling they run with the CPU clock)
    TIM4->ARR = 179;
}


void imu_timed_configure(I2C &i2c)
{
    uint8_t buf[3];
    i2c.smbus_write(IMU_DEVICE_ADDRESS, 0x20, 2, &config_20[0]);
    i2c.crude_hack();
    i2c.smbus_read(IMU_DEVICE_ADDRESS, 0x20, 2, &buf[0]);

    if (memcmp(&buf[0], &config_20[0], 2) != 0) {
        __asm__ volatile ("bkpt #01"); // failed to configure IMU
    }

    i2c.smbus_write(IMU_DEVICE_ADDRESS, 0x24, 3, &config_24[0]);
    i2c.crude_hack();
    i2c.smbus_read(IMU_DEVICE_ADDRESS, 0x24, 3, &buf[0]);

    if (memcmp(&buf[0], &config_24[0], 3) != 0) {
        __asm__ volatile ("bkpt #01"); // failed to configure IMU
    }

}


void imu_timed_enable()
{
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
}

ASYNC_CALLABLE imu_timed_full_buffer(
        const imu_buffer_t *&full_buffer,
        const imu_source_t source)
{
    return sources[source].wait_for_buffer(full_buffer);
}


void imu_timed_get_state(imu_source_t source_type,
                         uint16_t &last_seq,
                         uint16_t &last_timestamp)
{
    imu_source_data_t &source = sources[source_type];
    // XXX: this is a read-race
    last_seq = source.sample_seq;
    last_timestamp = source.last_sample.raw();
}



static inline void _sample_accelerometer()
{
    imu_source_data_t &source = sources[IMU_SOURCE_ACCELEROMETER];

    if (source.offset == IMU_BUFFER_LENGTH) {
        source.swap_buffers();
    }

    imu_buffer_t &buffer = source.buffer();

    i2c1.smbus_read_a(IMU_DEVICE_ADDRESS, IMU_ACCEL_ADDRESS,
                      3*sizeof(uint16_t),
                      (uint8_t*)&buffer.samples[source.offset++].vector[0]);

    source.last_sample = sched_clock::now();
    source.sample_seq += 1;
}


static inline void _sample_magnetometer()
{
    imu_source_data_t &source = sources[IMU_SOURCE_MAGNETOMETER];

    if (source.offset == IMU_BUFFER_LENGTH) {
        source.swap_buffers();
    }

    imu_buffer_t &buffer = source.buffer();

    i2c1.smbus_read_a(IMU_DEVICE_ADDRESS, IMU_COMPASS_ADDRESS,
                      3*sizeof(uint16_t),
                      (uint8_t*)&buffer.samples[source.offset++].vector[0]);

    source.last_sample = sched_clock::now();
    source.sample_seq += 1;
}


void TIM4_IRQHandler()
{
    TIM4->SR = 0;

    ctr += 1;
    if (ctr == 128 || ctr == 0) {
        // sample magnetometer
        _sample_magnetometer();
    } else if (ctr & 1) {
        // sample accelerometer
        _sample_accelerometer();
    }
}
