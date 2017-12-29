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

static constexpr std::uint8_t BME280_OK = 0x00;
static constexpr std::uint8_t BME280_ERR_INVALID_ID = 0x01;
static constexpr std::uint8_t BME280_ERR_CONFIG_VERIFY_FAILED = 0x02;
static constexpr std::uint8_t BME280_ERR_TIMEOUT = 0x03;

bool bme280_configure(I2C &i2c, uint8_t device_instance);


class BME280VerifiedWrite: public Coroutine
{
public:
    explicit BME280VerifiedWrite(I2C &bus):
        m_bus(bus)
    {

    }

private:
    I2C &m_bus;
    uint8_t m_device_addr;
    uint8_t m_reg_addr;
    uint8_t m_reg_value;
    uint8_t m_verify;
    uint8_t *m_status;
    bool m_timed_out;
    WaitFor m_wait_for;

public:
    void operator()(const uint8_t device_address,
                    const uint8_t register_address,
                    const uint8_t register_value,
                    uint8_t &status)
    {
        Coroutine::operator()();
        m_device_addr = device_address;
        m_reg_addr = register_address;
        m_reg_value = register_value;
        m_status = &status;
    }

    COROUTINE_DECL;
};


class BME280DetectAndConfigure: public Coroutine
{
public:
    explicit BME280DetectAndConfigure(I2C &bus):
        m_bus(bus),
        m_verified_write(m_bus)
    {

    }

private:
    I2C &m_bus;
    BME280VerifiedWrite m_verified_write;
    uint8_t m_device_address;
    uint8_t m_buf;
    uint8_t *m_status;
    bool m_was_present;
    bool m_timed_out;
    WaitFor m_wait_for;

public:
    void operator()(const uint8_t device_address,
                    uint8_t &status,
                    bool was_present = false)
    {
        Coroutine::operator()();
        m_device_address = device_address;
        m_status = &status;
        m_was_present = was_present;
    }

    COROUTINE_DECL;

};

#endif
