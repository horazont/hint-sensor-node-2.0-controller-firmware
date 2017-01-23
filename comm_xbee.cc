#include "comm_xbee.h"

#include <cstring>
#include <cstdio>


static CommXBEE *_xbee;


void _xbee_usart_rx_done_cb()
{
    _xbee->m_rx_state = CommXBEE::RX_FRAME_CHECKSUM;
}


void _xbee_usart_rx_data_cb(const uint8_t ch)
{
    switch (_xbee->m_rx_state) {
    case CommXBEE::RX_IDLE:
    {
        if (ch != 0x7e) {
            return;
        }
        _xbee->m_rx_state = CommXBEE::RX_FRAME_LENGTH_MSB;
        break;
    }
    case CommXBEE::RX_FRAME_LENGTH_MSB:
    {
        _xbee->m_rx_length = ch << 8;
        _xbee->m_rx_state = CommXBEE::RX_FRAME_LENGTH_LSB;
        break;
    }
    case CommXBEE::RX_FRAME_LENGTH_LSB:
    {
        _xbee->m_rx_length |= ch;
        _xbee->m_rx_state = CommXBEE::RX_FRAME_TYPE;
        break;
    }
    case CommXBEE::RX_FRAME_TYPE:
    {
        _xbee->m_rx_frame_type = ch;
        _xbee->m_rx_state = CommXBEE::RX_DATA;
        _xbee->m_usart.recv_a(&_xbee->m_rx_buffer[0], _xbee->m_rx_length-1,
                              _xbee_usart_rx_done_cb);
        break;
    }
    case CommXBEE::RX_DATA:
    {
        __asm__ volatile("bkpt #20"); // data cb called even though DMA should be going on
        break;
    }
    case CommXBEE::RX_FRAME_CHECKSUM:
    {
        _xbee->m_rx_state = CommXBEE::RX_IDLE;
        _xbee->m_rx_checksum = ch;
        if (_xbee->m_rx_frame_type == 0x8b) {
            _xbee->m_rx_packet_ready.trigger();
        }
        break;
    }
    }
}


CommXBEE::CommXBEE(USART &usart):
    m_usart(usart),
    m_buffer(),
    m_tx_seq(1)
{
    m_tx_sendv[0] = USART::sendv_item(&m_tx_frame_header[0], m_tx_frame_header.size());
    m_tx_sendv[2] = USART::sendv_item(&m_tx_checksum, 1);

    m_tx_frame_header[0] = 0x7e;
    m_tx_frame_header[1] = 0x00;
    m_tx_frame_header[2] = 0x00;
    m_tx_frame_header[3] = 0x10;
    m_tx_frame_header[4] = 0x00;
    std::memset(&m_tx_frame_header[5], 0, 8);
    m_tx_frame_header[13] = 0xff;
    m_tx_frame_header[14] = 0xfe;
    m_tx_frame_header[15] = 0x00;
    m_tx_frame_header[16] = 0x00;

    m_tx_checksum_base = 0;
    for (unsigned int i = 3; i < m_tx_frame_header.size(); ++i) {
        m_tx_checksum_base += m_tx_frame_header[i];
    }
}


ASYNC_CALLABLE CommXBEE::step(const sched_clock::time_point now)
{
    COROUTINE_INIT;
    if (_xbee) {
        __asm__ volatile("bkpt #20");
    }
    _xbee = this;
    m_usart.set_rx_callback(&_xbee_usart_rx_data_cb);

    while (1) {
        await(m_buffer.any_buffer_ready());
        m_tx_handle = m_buffer.dequeue(m_tx_buffer, m_tx_len, m_tx_flags);
        if (m_tx_handle == buffer_t::INVALID_BUFFER) {
            continue;
        }

        {
            uint16_t frame_length = m_tx_len + m_tx_frame_header.size() - 3;
            m_tx_frame_header[1] = (frame_length >> 8);
            m_tx_frame_header[2] = frame_length;
            m_tx_frame_header[4] = m_tx_seq;

            if (m_tx_flags & PRIO_NO_RETRIES) {
                m_tx_frame_header[16] = 0x01;
            } else {
                m_tx_frame_header[16] = 0;
            }

            uint8_t checksum = m_tx_checksum_base
                    + m_tx_frame_header[4]
                    + m_tx_frame_header[16];
            for (uint8_t i = 0; i < m_tx_len; ++i) {
                checksum += m_tx_buffer[i];
            }

            m_tx_checksum = 0xff - checksum;
        }

        m_tx_sendv[1] = USART::sendv_item(m_tx_buffer, m_tx_len);

        await(m_usart.tx_ready());
        await(m_usart.sendv_c(m_tx_sendv));
        m_buffer.release(m_tx_handle);
        m_tx_handle = buffer_t::INVALID_BUFFER;

        // wait for TX status
        //await(m_rx_packet_ready.ready_c());
        //m_rx_packet_ready.reset();
    }
    COROUTINE_END;
}
