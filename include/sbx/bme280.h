#ifndef BME280_H
#define BME280_H

#include <cstdint>

#include "i2clib.h"

static constexpr std::uint8_t BME280_DEVICE_ADDRESS = 0x76;
static constexpr std::uint8_t BME280_ID_REGISTER = 0xd0;
static constexpr std::uint8_t BME280_CTRL_HUM_REGISTER = 0xf2;
static constexpr std::uint8_t BME280_CTRL_MEAS_REGISTER = 0xf4;
static constexpr std::uint8_t BME280_CONFIG_REGISTER = 0xf5;
static constexpr std::uint8_t BME280_DATA_START = 0xf7;

void bme280_configure(I2C &i2c);

#endif
