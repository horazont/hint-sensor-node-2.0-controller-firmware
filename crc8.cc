#include "crc8.h"

CRC8::CRC8(uint8_t poly):
    m_poly(poly),
    m_crc(0)
{

}

void CRC8::reset()
{
    m_crc = 0;
}

uint8_t CRC8::feed(const uint8_t *data, std::size_t length)
{
    for (std::size_t i = 0; i < length; ++i) {
        feed(data[i]);
    }
    return m_crc;
}

uint8_t CRC8::feed(const uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        if (((m_crc >> 7) & 1) != ((byte >> i) & 1)) {
            m_crc = (m_crc << 1) ^ m_poly;
        } else {
            m_crc = m_crc << 1;
        }
    }
    return m_crc;
}
