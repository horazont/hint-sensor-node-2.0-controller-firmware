#ifndef CRC8_H
#define CRC8_H

#include <cstdint>


class CRC8
{
public:
    CRC8() = delete;
    explicit CRC8(uint8_t poly);
    CRC8(const CRC8 &ref) = delete;
    CRC8(CRC8 &&src) = delete;
    CRC8 &operator=(const CRC8 &ref) = delete;
    CRC8 &operator=(CRC8 &&src) = delete;

private:
    const uint8_t m_poly;
    uint8_t m_crc;

public:
    void reset();
    uint8_t feed(const uint8_t *data, std::size_t length);
    uint8_t feed(const uint8_t byte);
    inline uint8_t crc() const
    {
        return m_crc;
    }

};

#endif
