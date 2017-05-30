#include "onewire.h"

#include <cstdint>
#include <cstring>

#include <stm32f10x.h>

#include "config.h"


#define ONEWIRE_DATA_BAUDRATE (100000)
#define ONEWIRE_CONTROL_BAUDRATE (9600)


#define onewire_reset(core, status_out) do {\
    core.enter_control_mode();\
    await(core.probe(0xf0));\
    core.enter_data_mode();\
    status_out = (core.echo() & 0xf0) < 0xf0 ? ONEWIRE_PRESENCE : ONEWIRE_EMPTY;\
    } while(0)

#define onewire_read(core, out) do {await(core.probe(0xff)); out = core.echo() == 0xff;}while(0)
#define onewire_write1(core) await(core.probe(0xff))
#define onewire_write0(core) await(core.probe(0x00))

const uint8_t ONEWIRE_SEARCH_REQ = 0xf0;
const uint8_t ONEWIRE_MATCH_ROM = 0x55;
const uint8_t ONEWIRE_CONVERT_T = 0x44;
const uint8_t ONEWIRE_READ_SCRATCHPAD = 0xbe;
const uint8_t ONEWIRE_SKIP_ROM = 0xcc;

static OnewireCore *_curr_onewire;


void _onewire_usart_data_cb(const uint8_t ch,
                            const uint16_t sr)
{
    (void)sr;
    _curr_onewire->m_echo = ch;
    _curr_onewire->m_echo_received.trigger();
    _curr_onewire->m_usart.set_rx_callback(nullptr);
    _curr_onewire = nullptr;
}


OnewireCoroutine::OnewireCoroutine(OnewireCore &core):
    m_core(core)
{

}


void OnewireReadBytes::operator()(uint8_t *buf, const uint16_t length)
{
    Coroutine::operator()();
    m_dest = buf;
    m_length = length;
    m_offset = 0;
}

ASYNC_CALLABLE OnewireReadBytes::step(stm32_clock::time_point now)
{
    COROUTINE_INIT;
    for (m_offset = 0; m_offset < m_length; ++m_offset) {
        m_byte_buf = 0;
        for (m_bit = 0; m_bit < 8; ++m_bit) {
            await(m_core.probe(0xff));
            if (m_core.echo() == 0xff) {
                m_byte_buf |= (1 << m_bit);
            }
        }
        m_dest[m_offset] = m_byte_buf;
    }
    COROUTINE_END;
}



OnewireWriteBytes::OnewireWriteBytes(OnewireCore &core):
    m_core(core)
{

}

void OnewireWriteBytes::operator()(const uint8_t *buf, const uint16_t length)
{
    Coroutine::operator()();
    m_src = buf;
    m_length = length;
    m_offset = 0;
}

ASYNC_CALLABLE OnewireWriteBytes::step(stm32_clock::time_point now)
{
    COROUTINE_INIT;
    for (m_offset = 0; m_offset < m_length; ++m_offset) {
        m_byte_buf = m_src[m_offset++];
        for (m_bit = 0; m_bit < 8; ++m_bit) {
            if (m_byte_buf & 0x1) {
                onewire_write1(m_core);
            } else {
                onewire_write0(m_core);
            }
            m_byte_buf >>= 1;
        }
    }
    COROUTINE_END;
}


OnewireFindNext::OnewireFindNext(OnewireCore &core):
    m_core(core)
{

}

void OnewireFindNext::operator()(onewire_addr_t &addr)
{
    Coroutine::operator ()();
    m_addr = &addr;
}

ASYNC_CALLABLE OnewireFindNext::step(const stm32_clock::time_point now)
{
    COROUTINE_INIT;
    onewire_reset(m_core, m_status);
    if (m_status != ONEWIRE_PRESENCE) {
        COROUTINE_RETURN;
    }

    await_call(m_core.write_bytes, &ONEWIRE_SEARCH_REQ, 1);

    m_previous_alternative_bit = 0xff;
    for (m_offs = 0; m_offs < ONEWIRE_ADDR_LEN*8; ++m_offs) {
        onewire_read(m_core, m_false_presence);
        m_false_presence = !m_false_presence;
        onewire_read(m_core, m_true_presence);
        m_true_presence = !m_true_presence;
        (void)m_false_presence;

        m_byteaddr = (m_offs & 0xf8) >> 3;
        m_bitoffs = m_offs & 0x07;

        m_prevbit = (*m_addr)[m_byteaddr] & (1<<m_bitoffs);
        if (!m_prevbit) {
            // note this as a possible position to return to on the second
            // iteration, as a higher value is possible here
            if (m_true_presence) {
                m_previous_alternative_bit = m_offs;
            }
            if (!m_false_presence) {
                // we can abort here; the device which possibly was at the
                // current address is not here anymore.
                break;
            }
        } else if (!m_true_presence) {
            // dito
            break;
        }

        if (m_prevbit) {
            onewire_write1(m_core);
        } else {
            onewire_write0(m_core);
        }
    }

    if (m_previous_alternative_bit == 0xff) {
        m_status = ONEWIRE_EMPTY;
        COROUTINE_RETURN;
    }

    // re-initialize the bus for searching the next device
    onewire_reset(m_core, m_status);
    if (m_status != ONEWIRE_PRESENCE) {
        COROUTINE_RETURN;
    }

    await_call(m_core.write_bytes, &ONEWIRE_SEARCH_REQ, 1);

    for (m_offs = 0; m_offs < m_previous_alternative_bit; ++m_offs) {
        // ignore the presence strobes, although we *could* use them as a safety
        // net. if the device isn’t there anymore, we’ll find out later.
        onewire_read(m_core, m_false_presence);
        onewire_read(m_core, m_true_presence);

        m_byteaddr = (m_offs & 0xf8) >> 3;
        m_bitoffs = m_offs & 0x07;

        if ((*m_addr)[m_byteaddr] & (1<<m_bitoffs)) {
            onewire_write1(m_core);
        } else {
            onewire_write0(m_core);
        }
    }

    // okay, we have the equal part of the address, let’s continue with the new
    // part.

    // not looking for 'false' devices
    onewire_read(m_core, m_false_presence);

    onewire_read(m_core, m_true_presence);
    if (m_true_presence) {
        // no device here (yes, those presence strobes are inverted)
        m_status = ONEWIRE_ERROR;
        COROUTINE_RETURN;
    }

    // go down the `true` route
    onewire_write1(m_core);

    {
        const uint8_t byteaddr = (m_previous_alternative_bit & 0xF8) >> 3;
        const uint8_t bitoffs = (m_previous_alternative_bit & 0x07);
        (*m_addr)[byteaddr] |= (1<<bitoffs);
    }

    for (m_offs = m_previous_alternative_bit+1;
         m_offs < ONEWIRE_ADDR_LEN*8;
         ++m_offs)
    {
        onewire_read(m_core, m_false_presence);
        m_false_presence = !m_false_presence;
        onewire_read(m_core, m_true_presence);
        m_true_presence = !m_true_presence;

        m_byteaddr = (m_offs & 0xf8) >> 3;
        m_bitoffs = m_offs & 0x07;

        // we must go for false presence first (lower value)
        if (m_false_presence) {
            (*m_addr)[m_byteaddr] &= ~(1<<m_bitoffs);
            onewire_write0(m_core);
        } else if (m_true_presence) {
            (*m_addr)[m_byteaddr] |= (1<<m_bitoffs);
            onewire_write1(m_core);
        } else {
            // device vanished
            m_status = ONEWIRE_ERROR;
            COROUTINE_RETURN;
        }
    }

    m_status = ONEWIRE_PRESENCE;

    COROUTINE_END;
}


void OnewireSelectDevice::operator()(const onewire_addr_t &addr)
{
    Coroutine::operator()();
    m_addr = &addr;
}

ASYNC_CALLABLE OnewireSelectDevice::step(const stm32_clock::time_point now)
{
    COROUTINE_INIT;
    onewire_reset(m_core, m_status);
    if (m_status != ONEWIRE_PRESENCE) {
        COROUTINE_RETURN;
    }

    await_call(m_core.write_bytes, &ONEWIRE_MATCH_ROM, 1);

    for (m_byte = 0; m_byte < ONEWIRE_ADDR_LEN; m_byte++) {
        for (m_bit = 0; m_bit < 8; m_bit++) {
            if (((*m_addr)[m_byte] & (1<<m_bit)) != 0) {
                onewire_write1(m_core);
            } else {
                onewire_write0(m_core);
            }
        }
    }

    m_status = ONEWIRE_PRESENCE;

    COROUTINE_END;
}



ASYNC_CALLABLE OnewireDetectDevices::step(const stm32_clock::time_point now)
{
    COROUTINE_INIT;
    onewire_reset(m_core, m_status);
    COROUTINE_END;
}



OnewireCore::OnewireCore(USART &usart):
    m_usart(usart),
    m_brr_control(m_usart.calc_brr(9600)),
    m_brr_data(m_usart.calc_brr(115200)),
    read_bytes(*this),
    write_bytes(*this),
    find_next(*this),
    select_device(*this),
    detect_devices(*this)
{

}

ASYNC_CALLABLE OnewireCore::probe(const uint8_t signal)
{
#ifndef NDEBUG
    if (_curr_onewire && _curr_onewire != this) {
        __asm__ volatile ("bkpt #01");  // conflict between two OnewireBase instances!
    }
#endif
    _curr_onewire = this;
    m_echo_received.reset();
    m_signal = signal;
    m_usart.set_rx_callback(&_onewire_usart_data_cb);
    m_usart.send_a(&m_signal, 1);
    return m_echo_received.ready_c();
}

void OnewireCore::enter_control_mode()
{
    m_usart.set_brr(m_brr_control);
}

void OnewireCore::enter_data_mode()
{
    m_usart.set_brr(m_brr_data);
}
