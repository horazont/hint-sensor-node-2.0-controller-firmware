#ifndef COMM_XBEE_H
#define COMM_XBEE_H

#include "comm.h"
#include "coroutine.h"
#include "usart.h"

class CommXBEE: public Coroutine
{
private:
    enum RXState: uint8_t
    {
        RX_IDLE = 0,
        RX_FRAME_LENGTH_MSB,
        RX_FRAME_LENGTH_LSB,
        RX_FRAME_TYPE,
        RX_DATA,
        RX_FRAME_CHECKSUM
    };

public:
    using buffer_t = CommBuffer<4096, 84>;

public:
    CommXBEE() = delete;
    explicit CommXBEE(USART &usart);

private:
    USART &m_usart;
    buffer_t m_buffer;

    uint8_t *m_tx_buffer;
    buffer_t::buffer_handle_t m_tx_handle;
    uint16_t m_tx_len;
    uint8_t m_tx_seq;
    uint8_t m_tx_flags;
    std::array<uint8_t, 17> m_tx_frame_header;
    uint8_t m_tx_checksum;
    uint8_t m_tx_checksum_base;
    USART::sendv_array_t m_tx_sendv;
    notifier_t m_tx_acked;

    RXState m_rx_state;
    uint16_t m_rx_length;
    uint8_t m_rx_frame_type;
    uint8_t m_rx_frame_id;
    uint8_t m_rx_checksum;
    notifier_t m_rx_packet_ready;
    std::array<uint8_t, 128> m_rx_buffer;

public:
    void operator()()
    {
        Coroutine::operator()();
    }

    inline buffer_t &output_buffer()
    {
        return m_buffer;
    }

    COROUTINE_DECL;

    friend void _xbee_usart_rx_data_cb(const uint8_t ch);
    friend void _xbee_usart_rx_done_cb();
};

#endif // COMM_XBEE_H
