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

static const uint16_t cfg_timeout = 10;


COROUTINE_DEF(BME280VerifiedWrite)
{
    COROUTINE_INIT;

    // For some reason, the coroutine version isn’t working reliably.

    await_call(m_wait_for, m_bus.smbus_writec(m_device_addr, m_reg_addr, 1, &m_reg_value), cfg_timeout, m_timed_out);
    if (m_timed_out) {
        *m_status = BME280_ERR_TIMEOUT;
        COROUTINE_RETURN;
    }

    // crude hack 2.0
    yield;
    m_bus.disable();
    m_bus.init();
    m_bus.enable();
    yield;

    m_verify = ~m_reg_value;
    await_call(m_wait_for, m_bus.smbus_readc(m_device_addr, m_reg_addr, 1, &m_verify), cfg_timeout, m_timed_out);
    if (m_timed_out) {
        *m_status = BME280_ERR_TIMEOUT;
        COROUTINE_RETURN;
    }

    if (m_verify == m_reg_value) {
        *m_status = BME280_OK;
    } else {
        *m_status = BME280_ERR_CONFIG_VERIFY_FAILED;
    }

    COROUTINE_END;
}


COROUTINE_DEF(BME280DetectAndConfigure)
{
    COROUTINE_INIT;
    m_buf = 0x00;
    await_call(m_wait_for, m_bus.smbus_readc(m_device_address, BME280_ID_REGISTER, 1, &m_buf), cfg_timeout, m_timed_out);
    if (m_timed_out) {
        *m_status = BME280_ERR_TIMEOUT;
        // try to re-initialise the bus, because this is suspicious

        yield;
        m_bus.disable();
        m_bus.init();
        m_bus.enable();
        yield;

        COROUTINE_RETURN;
    }
    if (m_bus.last_err() != 0) {
        *m_status = BME280_ERR_BUS_ERROR;
        COROUTINE_RETURN;
    }
    if (m_buf != 0x60) {
        *m_status = BME280_ERR_INVALID_ID;
        COROUTINE_RETURN;
    }

    if (m_was_present) {
        *m_status = BME280_OK;
        COROUTINE_RETURN;
    }

    await_call(m_verified_write, m_device_address, BME280_CONFIG_REGISTER, reg_config, *m_status);
    if (*m_status != BME280_OK) {
        COROUTINE_RETURN;
    }

    await_call(m_verified_write, m_device_address, BME280_CTRL_HUM_REGISTER, reg_ctrl_hum, *m_status);
    if (*m_status != BME280_OK) {
        COROUTINE_RETURN;
    }

    await_call(m_verified_write, m_device_address, BME280_CTRL_MEAS_REGISTER, reg_ctrl_meas, *m_status);
    if (*m_status != BME280_OK) {
        COROUTINE_RETURN;
    }

    COROUTINE_END;
}
