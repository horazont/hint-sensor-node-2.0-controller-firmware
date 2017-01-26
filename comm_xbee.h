#ifndef COMM_XBEE_H
#define COMM_XBEE_H

#include "comm.h"
#include "coroutine.h"
#include "usart.h"

class CommXBEE: public Coroutine
{
public:
    using buffer_t = CommBuffer<4096, 84>;

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

    struct tx_info_t
    {
        buffer_t::buffer_handle_t handle;
        uint8_t *buffer;
        uint16_t len;
        uint8_t flags;
        std::array<uint8_t, 17> frame_header;
        uint8_t checksum;
        notifier_t acked;
        USART::sendv_array_t sendv;
    };

    static constexpr uint8_t FRAME_SEQNR_BASE = 0xf8;

public:
    CommXBEE() = delete;
    explicit CommXBEE(USART &usart);

private:
    USART &m_usart;
    buffer_t m_buffer;

    std::array<tx_info_t, 2> m_tx;
    uint8_t m_tx_curr_buf;
    uint8_t m_tx_prev_buf;

    uint8_t m_tx_checksum_base;

    WakeupCondition m_tx_done;

    RXState m_rx_state;
    uint16_t m_rx_length;
    uint8_t m_rx_frame_type;
    uint8_t m_rx_frame_id;
    uint8_t m_rx_checksum;
    notifier_t m_rx_packet_ready;
    std::array<uint8_t, 128> m_rx_buffer;
    bool m_rx_spurious_rxneie;

public:
    void operator()()
    {
        Coroutine::operator()();
        m_tx_done = WakeupCondition::none();
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
