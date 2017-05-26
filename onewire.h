#ifndef ONEWIRE_H
#define ONEWIRE_H

#include <array>

#include "coroutine.h"
#include "usart.h"

static constexpr unsigned int ONEWIRE_ADDR_LEN = 8;

using onewire_addr_t = uint8_t[ONEWIRE_ADDR_LEN];


enum onewire_status_t {
    ONEWIRE_NCONN = 0,
    ONEWIRE_PRESENCE = 1,
    ONEWIRE_ERROR = 2,
    ONEWIRE_EMPTY = 3
};


class OnewireCore
{
public:
    explicit OnewireCore(USART &usart);

private:
    USART &m_usart;
    const uint16_t m_brr_control;
    const uint16_t m_brr_data;
    notifier_t m_echo_received;
    uint8_t m_echo;
    uint8_t m_signal;

public:
    ASYNC_CALLABLE probe(const uint8_t signal);
    void enter_control_mode();
    void enter_data_mode();

    inline uint8_t echo() const
    {
        return m_echo;
    }

public:
    friend void _onewire_usart_data_cb(const uint8_t dr, const uint16_t sr);

};


class OnewireReadBytes: public Coroutine
{
public:
    OnewireReadBytes(OnewireCore &core);

private:
    OnewireCore &m_core;
    uint8_t *m_dest;
    uint16_t m_offset;
    uint16_t m_length;
    uint8_t m_bit;
    uint8_t m_byte_buf;

public:
    void operator()(uint8_t *buf, const uint16_t length);
    COROUTINE_DECL;

};


class OnewireWriteBytes: public Coroutine
{
public:
    OnewireWriteBytes(OnewireCore &core);

private:
    OnewireCore &m_core;
    const uint8_t *m_src;
    uint16_t m_offset;
    uint16_t m_length;
    uint8_t m_bit;
    uint8_t m_byte_buf;

public:
    void operator()(const uint8_t *buf, const uint16_t length);
    COROUTINE_DECL;

};



class OnewireFindNext: public Coroutine
{
public:
    explicit OnewireFindNext(OnewireCore &core);

private:
    OnewireCore &m_core;
    OnewireWriteBytes m_write_bytes;

private:
    onewire_addr_t *m_addr;
    onewire_status_t m_status;
    uint8_t m_previous_alternative_bit;
    uint_fast8_t m_offs;

    uint8_t m_false_presence;
    uint8_t m_true_presence;

    uint8_t m_byteaddr;
    uint8_t m_bitoffs;
    uint8_t m_prevbit;

public:
    void operator()(onewire_addr_t &addr);
    COROUTINE_DECL;

    inline onewire_status_t status() const
    {
        return m_status;
    }

};


#endif // ONEWIRE_H
