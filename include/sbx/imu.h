#ifndef IMU_H
#define IMU_H

/**
 * LSM303D interface module.
 *
 * We sample the LSM303D accelerometer at 200 Hz and the Magnetometer at merely
 * 3.125 Hz. We found that magnetometer data is very stable and not that
 * interesting; we can save the bandwidth for more on the magnetometer channel.
 */

#include <array>
#include <cstdint>

#include "coroutine.h"
#include "i2clib.h"

struct __attribute__((packed)) imu_data_point_t
{
    uint16_t vector[3];
};

static constexpr std::size_t IMU_BUFFER_LENGTH = 12;

struct imu_buffer_t
{
    uint16_t first_sample;
    std::array<imu_data_point_t, IMU_BUFFER_LENGTH> samples;
};

struct bme280_buffer_t
{
    sched_clock::time_point timestamp;
    std::array<uint8_t, 8> data;
};

enum imu_source_t: uint8_t {
    IMU_SOURCE_ACCELEROMETER = 0,
    IMU_SOURCE_MAGNETOMETER = 1
};

void imu_timed_init();
void imu_timed_configure(I2C &i2c);
void bme280_configure(I2C &i2c);
void imu_timed_enable();

ASYNC_CALLABLE imu_timed_full_buffer(
        const imu_buffer_t *&full_buffer,
        const imu_source_t source);

ASYNC_CALLABLE bme280_read(bme280_buffer_t &dest);

void imu_timed_get_state(imu_source_t source_type,
                         uint16_t &last_seq,
                         uint16_t &last_timestamp);

#ifdef __cplusplus
extern "C" {
#endif

void TIM4_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // IMU_H
