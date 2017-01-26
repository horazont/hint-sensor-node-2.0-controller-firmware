#include "imu.h"

#include <stm32f10x.h>

#include "i2clib.h"
#include "notify.h"

static std::array<notifier_t, 2> _notifiers;
static std::array<imu_buffer_t, 2> _buffers;
static volatile uint_fast8_t _current_notifier;
static uint_fast8_t _offset;
static bool _accel;
static uint16_t _seq;

static constexpr uint8_t IMU_DEVICE_ADDRESS = 0x1d;
static constexpr uint8_t IMU_ACCEL_ADDRESS = 0x28;
static constexpr uint8_t IMU_COMPASS_ADDRESS = 0x08;

void imu_timed_init()
{
    _current_notifier = 0;
    _offset = 0;
    _accel = true;
    _seq = 0;

    TIM4->CR1 = 0;
    TIM4->CR2 = 0;
    TIM4->SMCR = 0;
    TIM4->DIER = TIM_DIER_UIE;

    // 5ms period -- we read the accelerometer and compass alternatingly
    TIM4->PSC = 1000;
    // those fucking timer clocks are doubled!
    TIM4->ARR = 360;
}

void imu_timed_enable()
{
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);
}

ASYNC_CALLABLE imu_timed_full_buffer(const imu_buffer_t *&full_buffer)
{
    uint_fast8_t curr_notifier = _current_notifier;
    full_buffer = &_buffers[curr_notifier];
    return _notifiers[curr_notifier].ready_c();
}

static void _swap_buffers()
{
    uint_fast8_t curr_notifier = _current_notifier;
    _buffers[curr_notifier].seq = _seq++;
    _notifiers[curr_notifier].trigger();
    curr_notifier ^= 1;
    _notifiers[curr_notifier].reset();
    _current_notifier = curr_notifier;
    _offset = 0;
}

static inline imu_buffer_t &_curr_buffer()
{
    return _buffers[_current_notifier];
}


void TIM4_IRQHandler()
{
    TIM4->SR = 0;

    if (_offset == IMU_BUFFER_LENGTH) {
        _swap_buffers();
    }

    imu_buffer_t &buffer = _curr_buffer();
    if (_accel) {
        i2c_smbus_read_a(IMU_DEVICE_ADDRESS, IMU_ACCEL_ADDRESS,
                       3*sizeof(uint16_t),
                       (uint8_t*)&buffer.samples[_offset].accel_compass[0]);
        _accel = !_accel;
    } else {
        i2c_smbus_read_a(IMU_DEVICE_ADDRESS, IMU_COMPASS_ADDRESS,
                       3*sizeof(uint16_t),
                       (uint8_t*)&buffer.samples[_offset].accel_compass[3]);
        _accel = !_accel;
        _offset += 1;
    }
}
