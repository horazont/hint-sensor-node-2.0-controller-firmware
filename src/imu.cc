#include "imu.h"

#include <stm32f10x.h>

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

void imu_timed_init()
{
    ctr = 0;

    TIM4->CR1 = 0;
    TIM4->CR2 = 0;
    TIM4->SMCR = 0;
    TIM4->DIER = TIM_DIER_UIE;

    // 2.5ms period
    TIM4->PSC = 1000;
    // those fucking timer clocks are doubled!
    TIM4->ARR = 180;
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
    if (ctr == 128) {
        // sample magnetometer
        _sample_magnetometer();
        ctr = 0;
    } else if (ctr & 1) {
        // sample accelerometer
        _sample_accelerometer();
    }
}
