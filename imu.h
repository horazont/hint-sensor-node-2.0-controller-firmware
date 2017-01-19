#ifndef IMU_H
#define IMU_H

#include <array>
#include <cstdint>

#include "coroutine.h"

struct __attribute__((packed)) imu_data_point_t
{
    uint16_t accel_compass[6];
};

static constexpr std::size_t IMU_BUFFER_LENGTH = 12;

using imu_buffer_t = std::array<imu_data_point_t, IMU_BUFFER_LENGTH>;

void imu_timed_init();
void imu_timed_enable();

ASYNC_CALLABLE imu_timed_full_buffer(const imu_buffer_t *&full_buffer);

#ifdef __cplusplus
extern "C" {
#endif

void TIM4_IRQHandler();

#ifdef __cplusplus
}
#endif

#endif // IMU_H
