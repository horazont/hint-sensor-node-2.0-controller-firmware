#include "bme280.h"


static const uint8_t reg_config =
        // to address 0xf5
        0b0 |  // no SPI 3w mode
        (0b001 << 2) |  // filter coefficient 2
        (0b101 << 5)  // 1000 ms standby time
        ;

static const uint8_t reg_ctrl_hum =
        // to address f2
        0b010  // oversample humidity x2
        ;

static const uint8_t reg_ctrl_meas =
        // to address f4
        0b11 |  // normal mode
        (0b101 << 2) |  // oversample pressure x16
        (0b010 << 5)  // oversample temperature x2
        ;

void _configure_verified(I2C &i2c, const uint8_t reg_addr, uint8_t value)
{
    i2c.smbus_write(BME280_DEVICE_ADDRESS, reg_addr, 1, &value);
    // reset the I2C peripherial due to write weirdness
    i2c.crude_hack();

    uint8_t verification = 0;
    i2c.smbus_read(BME280_DEVICE_ADDRESS, reg_addr, 1, &verification);

    if (verification != value) {
        __asm__ volatile("bkpt #01");  // BME280 configuration failed
    }
}


void bme280_configure(I2C &i2c)
{
    uint8_t id = 0x00;
    i2c.smbus_read(BME280_DEVICE_ADDRESS, BME280_ID_REGISTER, 1, &id);
    if (id != 0x60) {
        __asm__ volatile ("bkpt #01");  // BME280 does not reply with correct id
    }

    i2c.smbus_read(BME280_DEVICE_ADDRESS, BME280_ID_REGISTER, 1, &id);
    if (id != 0x60) {
        __asm__ volatile ("bkpt #01");  // BME280 does not reply with correct id
    }

    _configure_verified(i2c, BME280_CONFIG_REGISTER, reg_config);
    _configure_verified(i2c, BME280_CTRL_HUM_REGISTER, reg_ctrl_hum);
    _configure_verified(i2c, BME280_CTRL_MEAS_REGISTER, reg_ctrl_meas);
}
