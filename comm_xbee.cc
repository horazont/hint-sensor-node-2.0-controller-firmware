#include "comm_xbee.h"

#include <cstring>
#include <cstdio>


CommXBEERX *CommXBEERX::m_xbee = nullptr;


uint8_t calculate_checksum(const uint8_t *buf, const uint16_t len)
{
    uint8_t tmp = 0;
    for (unsigned int i = 0; i < len; ++i) {
        tmp += buf[i];
    }
    return 0xff - tmp;
}


CommXBEETX::CommXBEETX(USART &usart):
    m_usart(usart),
    m_buffer(),
    m_tx_curr_buf(0),
    m_tx_prev_buf(1)
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


ASYNC_CALLABLE CommXBEETX::step(const sched_clock::time_point now)
{
    COROUTINE_INIT;
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
        /*
        await(usart2.send_c(&m_tx[m_tx_curr_buf].frame_header[4], 1));
        await(usart2.send_c(&m_tx[m_tx_curr_buf].checksum, 1));
        */
        if (m_tx[m_tx_prev_buf].handle != buffer_t::INVALID_BUFFER) {
            // we have to wait for the previous transmission to complete first
            await(m_tx_done);
            // release the kraken^Wbuffer!
            m_buffer.release(m_tx[m_tx_prev_buf].handle);
            m_tx[m_tx_prev_buf].handle = buffer_t::INVALID_BUFFER;
        }

        // wait for the transmitter to become ready
        // await(m_usart.tx_ready());
        // send asynchronously and prep next packet if possible
        m_tx_done = m_usart.sendv_c(m_tx[m_tx_curr_buf].sendv);
        m_tx_prev_buf = m_tx_curr_buf;
        m_tx_curr_buf = m_tx_curr_buf ^ 1;

    }
    COROUTINE_END;
}


CommXBEERX::CommXBEERX(USART &usart):
    m_usart(usart),
    m_overruns(0),
    m_checksum_errors(0),
    m_errors(0),
    m_unknown_frames(0),
    m_skipped_bytes(0)
{

}

bool CommXBEERX::handle_frame()
{
    // frame type
    switch (m_buf[0]) {
    case 0x8b:  // transmit status
    {
        const uint8_t retries = m_buf[4];
        const uint8_t status = m_buf[5];
        m_tx_retries += retries;
        if (status != 0) {
            m_tx_lost += 1;
        }
        break;
    }
    default:
    {
        m_unknown_frames += 1;
        break;
    }
    }

    return true;
}

void CommXBEERX::set_rx_failed()
{
    m_xbee->m_errors += 1;
    if (m_xbee->m_interrupt_state.handle != buffer_t::INVALID_BUFFER) {
        m_xbee->m_buffer.release(m_xbee->m_interrupt_state.handle);
    }
    m_xbee->m_interrupt_state.state = CommXBEERX::RX_IDLE;
}

void CommXBEERX::rx_done_cb(bool success)
{
    if (!success) {
        m_xbee->set_rx_failed();
    } else {
        m_xbee->m_buffer.set_ready(m_xbee->m_interrupt_state.handle);
    }
    m_xbee->m_interrupt_state.handle = buffer_t::INVALID_BUFFER;
    m_xbee->m_interrupt_state.state = CommXBEERX::RX_IDLE;
}

void CommXBEERX::rx_data_cb(const uint8_t ch, const uint16_t sr)
{
    if (sr & USART_SR_FE || sr & USART_SR_NE) {
        m_xbee->set_rx_failed();
        return;
    }
    switch (m_xbee->m_interrupt_state.state) {
    case CommXBEERX::RX_IDLE:
    {
        if (ch != 0x7e) {
            m_xbee->m_skipped_bytes += 1;
            return;
        }
        m_xbee->m_interrupt_state.state = CommXBEERX::RX_FRAME_LENGTH_MSB;
        m_xbee->m_interrupt_state.length = 0;
        break;
    }
    case CommXBEERX::RX_FRAME_LENGTH_MSB:
    {
        m_xbee->m_interrupt_state.length = ch << 8;
        m_xbee->m_interrupt_state.state = CommXBEERX::RX_FRAME_LENGTH_LSB;
        break;
    }
    case CommXBEERX::RX_FRAME_LENGTH_LSB:
    {
        m_xbee->m_interrupt_state.length |= ch;
        if (m_xbee->m_interrupt_state.length > buffer_t::BUFFER_SIZE-1) {
            // this is most likely a lost byte, and we somehow locked onto
            // a part of another packet, which can happen in rare cases.
            // we should abort reception immediately and wait for the next
            // packet to lock at.
            m_xbee->m_overruns += 1;
            m_xbee->set_rx_failed();
            m_xbee->m_interrupt_state.state = CommXBEERX::RX_IDLE;
            break;
        }
        uint8_t *buf;
        m_xbee->m_interrupt_state.handle = m_xbee->m_buffer.allocate(
                    buf,
                    m_xbee->m_interrupt_state.length+1
                    );
        if (m_xbee->m_interrupt_state.handle == buffer_t::INVALID_BUFFER) {
            m_xbee->set_rx_failed();
            m_xbee->m_overruns += 1;
            m_xbee->m_interrupt_state.state = CommXBEERX::RX_IDLE;
            break;
        }

        m_xbee->m_usart.recv_a(buf, m_xbee->m_interrupt_state.length+1,
                               rx_done_cb);
        m_xbee->m_interrupt_state.spurious_rxneie = false;

        m_xbee->m_interrupt_state.state = CommXBEERX::RX_DATA;
        break;
    }
    case CommXBEERX::RX_DATA:
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
        if (m_xbee->m_interrupt_state.spurious_rxneie) {
            __asm__ volatile("bkpt #20"); // data cb called even though DMA should be going on
        } else {
            m_xbee->m_interrupt_state.spurious_rxneie = true;
        }
        break;
    }
    }
}

ASYNC_CALLABLE CommXBEERX::step(const stm32_clock::time_point now)
{
    COROUTINE_INIT;
    if (m_xbee) {
        __asm__ volatile("bkpt #20");
    }
    m_xbee = this;
    m_usart.set_rx_callback(&rx_data_cb);

    while (1) {
        await(m_buffer.any_buffer_ready());
        m_handle = m_buffer.dequeue(m_buf, m_length, m_flags);
        if (m_handle == buffer_t::INVALID_BUFFER) {
            continue;
        }

        m_expected_checksum = calculate_checksum(m_buf, m_length-1);
        if (m_buf[m_length-1] != m_expected_checksum) {
            m_checksum_errors += 1;
        }

        if (handle_frame()) {
            m_buffer.release(m_handle);
        }
    }
    COROUTINE_END;
}
