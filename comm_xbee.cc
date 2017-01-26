#include "comm_xbee.h"

#include <cstring>
#include <cstdio>


static CommXBEE *_xbee;


void _xbee_usart_rx_done_cb(bool success)
{
    if (success) {
        _xbee->m_rx_state = CommXBEE::RX_FRAME_CHECKSUM;
    } else {
        _xbee->m_rx_errors += 1;
        _xbee->m_rx_state = CommXBEE::RX_IDLE;
    }
}


void _xbee_usart_rx_data_cb(const uint8_t ch, const uint16_t sr)
{
    if (sr & USART_SR_FE || sr & USART_SR_NE) {
        _xbee->m_rx_errors += 1;
        _xbee->m_rx_state = CommXBEE::RX_IDLE;
        return;
    }
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
        if (_xbee->m_rx_length > 128) {
            // this is most likely a lost byte, and we somehow locked onto
            // a part of another packet, which can happen in rare cases.
            // we should abort reception immediately and wait for the next
            // packet to lock at.
            _xbee->m_rx_overruns += 1;
            _xbee->m_rx_state = CommXBEE::RX_IDLE;
            break;
        }
        _xbee->m_rx_state = CommXBEE::RX_FRAME_TYPE;
        break;
    }
    case CommXBEE::RX_FRAME_TYPE:
    {
        _xbee->m_rx_frame_type = ch;
        _xbee->m_rx_state = CommXBEE::RX_DATA;
        _xbee->m_usart.recv_a(&_xbee->m_rx_buffer[0], _xbee->m_rx_length-1,
                              _xbee_usart_rx_done_cb);
        _xbee->m_rx_spurious_rxneie = false;
        break;
    }
    case CommXBEE::RX_DATA:
    {
        /**
         * I’m a bit lost here. I’m not sure what to do and how this interrupt
         * can be triggered at all if DMA is going on and RXNEIE is not enabled.
         *
         * I think a possible condition is that enabling of DMA is preempted by
         * an interrupt which takes a few more cycles to execute. In that case,
         * it might be that the interrupt is set pending before DMA can take up
         * work.
         *
         * If that’s the case, we simply should ignore that. Let’s add a boolean
         * which tracks spurious interrupts and only triggers the debug
         * condition if it happens twice.
         */
        if (_xbee->m_rx_spurious_rxneie) {
            __asm__ volatile("bkpt #20"); // data cb called even though DMA should be going on
        } else {
            _xbee->m_rx_spurious_rxneie = true;
        }
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
    m_tx_curr_buf(0),
    m_tx_prev_buf(1),
    m_rx_overruns(0),
    m_rx_errors(0)
{
    for (tx_info_t &info: m_tx) {
        info.handle = buffer_t::INVALID_BUFFER;

        info.frame_header[0] = 0x7e;
        info.frame_header[1] = 0x00;
        info.frame_header[2] = 0x00;
        info.frame_header[3] = 0x10;
        info.frame_header[4] = 0x00;
        std::memset(&info.frame_header[5], 0, 8);
        info.frame_header[13] = 0xff;
        info.frame_header[14] = 0xfe;
        info.frame_header[15] = 0x00;
        info.frame_header[16] = 0x00;

        info.sendv[0] = USART::sendv_item(&info.frame_header[0], info.frame_header.size());
        info.sendv[2] = USART::sendv_item(&info.checksum, 1);
    }

    m_tx_checksum_base = 0;
    for (unsigned int i = 3; i < m_tx[0].frame_header.size(); ++i) {
        m_tx_checksum_base += m_tx[0].frame_header[i];
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

    m_tx_curr_buf = 0;

    while (1) {
        await(m_buffer.any_buffer_ready());
        m_tx[m_tx_curr_buf].handle = m_buffer.dequeue(
                    m_tx[m_tx_curr_buf].buffer,
                    m_tx[m_tx_curr_buf].len,
                    m_tx[m_tx_curr_buf].flags);
        if (m_tx[m_tx_curr_buf].handle == buffer_t::INVALID_BUFFER) {
            continue;
        }

        {
            tx_info_t &info = m_tx[m_tx_curr_buf];
            uint16_t frame_length = info.len + info.frame_header.size() - 3;
            info.frame_header[1] = (frame_length >> 8);
            info.frame_header[2] = frame_length;
            info.frame_header[4] = FRAME_SEQNR_BASE + m_tx_curr_buf;

            if (info.flags & PRIO_NO_RETRIES) {
                info.frame_header[16] = 0x01;
            } else {
                info.frame_header[16] = 0;
            }

            uint8_t checksum = m_tx_checksum_base
                    + info.frame_header[4]
                    + info.frame_header[16];
            for (uint8_t i = 0; i < info.len; ++i) {
                checksum += info.buffer[i];
            }

            info.checksum = 0xff - checksum;

            info.sendv[1] = USART::sendv_item(info.buffer, info.len);
        }

        if (m_tx[m_tx_prev_buf].handle != buffer_t::INVALID_BUFFER) {
            // we have to wait for the previous transmission to complete first
            await(m_tx_done);
            // release the kraken^Wbuffer!
            m_buffer.release(m_tx[m_tx_prev_buf].handle);
            m_tx[m_tx_prev_buf].handle = buffer_t::INVALID_BUFFER;
        }

        // wait for the transmitter to become ready
        await(m_usart.tx_ready());
        // send asynchronously and prep next packet if possible
        m_tx_done = m_usart.sendv_c(m_tx[m_tx_curr_buf].sendv);
        m_tx_prev_buf = m_tx_curr_buf;
        m_tx_curr_buf = m_tx_curr_buf ^ 1;

    }
    COROUTINE_END;
}
