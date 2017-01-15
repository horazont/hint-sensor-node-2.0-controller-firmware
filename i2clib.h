#ifndef SBX_I2CLIB_H
#define SBX_I2CLIB_H

#include <stdbool.h>

#include <stm32f10x.h>

void i2c_init();
void i2c_workaround_reset();
void i2c_enable();
void i2c_disable();

bool i2c_is_busy();

bool i2c_smbus_read(const uint8_t device_address,
                    const uint8_t register_address,
                    const uint8_t nbytes,
                    uint8_t *buf);
bool i2c_smbus_write(const uint8_t device_address,
                     const uint8_t register_address,
                     const uint8_t nbytes,
                     const uint8_t *buf);

#endif
