#ifndef COMM_SBX_H
#define COMM_SBX_H

#include <cstdint>

#ifdef XBEE_USE_ENCRYPTION
static constexpr std::size_t MAX_XBEE_PAYLOAD_SIZE = 66;
#else
static constexpr std::size_t MAX_XBEE_PAYLOAD_SIZE = 84;
#endif

#define SBX_LIGHT_SENSOR_SAMPLES (6)
#define SBX_LIGHT_SENSOR_CHANNELS (4)

#define COMM_PACKED __attribute__((packed))

enum class sbx_msg_type: std::uint8_t
{
    PING = 0x01,

    HELLO = 0x80,
    PONG = 0x81,
    STATUS = 0x82,
    SENSOR_DS18B20 = 0xf1,
    SENSOR_NOISE = 0xf2,
    SENSOR_DHT = 0xf3,
    SENSOR_LIGHT = 0xf4,
    SENSOR_STREAM_ACCEL_X = 0xf8,
    SENSOR_STREAM_ACCEL_Y = 0xf9,
    SENSOR_STREAM_ACCEL_Z = 0xfa,
    SENSOR_STREAM_COMPASS_X = 0xfb,
    SENSOR_STREAM_COMPASS_Y = 0xfc,
    SENSOR_STREAM_COMPASS_Z = 0xfd,
    RESERVED = 0xff,
};


typedef std::uint16_t sbx_uptime_t;
typedef std::uint32_t sbx_rtc_t;

struct COMM_PACKED sbx_msg_sensor_stream_t
{
    /**
     * "Sequence number" to be able to identify short packet loss.
     */
    uint16_t seq;

    /**
     * Average sample value.
     */
    uint16_t average;

    /**
     * Coded samples.
     */
    uint8_t data[0];
};

struct COMM_PACKED sbx_msg_ds18b20_t
{
    /**
     * Milliseconds since boot.
     *
     * Wraps around every ~65 seconds or so. Since all sensors are sampled
     * more often than that, wraparound can be compensated for.
     */
    sbx_uptime_t timestamp;

    /**
     * DS18B20 sensor ID
     */
    uint8_t id[8];

    /**
     * Raw 16 bit sensor value
     */
    uint16_t raw_value;
};

struct COMM_PACKED sbx_msg_noise_t
{
    sbx_uptime_t timestamp;

    /**
     * Raw 16 bit sensor value
     */
    uint16_t raw_value;
};

struct COMM_PACKED sbx_msg_light_sample_t
{
    /**
     * Timestamp at which the first channel was sampled.
     *
     * The channels are sampled in undefined order with an interval of 200ms.
     */
    sbx_uptime_t timestamp;

    /**
     * Data for each of the four channels, in order: Red, Green, Blue, Clear.
     */
    uint16_t ch[4];
};

struct COMM_PACKED sbx_msg_light_t
{
    /**
     * Array containing the samples.
     *
     * Six samples are safe even with encryption and provide a reasonable update
     * interval.
     */
    sbx_msg_light_sample_t samples[6];
};

struct COMM_PACKED sbx_msg_hello_t
{
    sbx_rtc_t rtc;
    sbx_uptime_t uptime;
};

struct COMM_PACKED sbx_msg_status_t
{
    sbx_rtc_t rtc;
    sbx_uptime_t uptime;
    struct COMM_PACKED {
        uint16_t rx_errors;
        uint16_t rx_overruns;
        uint16_t tx_non_acked;
        uint16_t tx_retries;
    } xbee_status;
    struct COMM_PACKED {
        uint8_t undervoltage_detected;
    } core_status;
};

struct COMM_PACKED sbx_msg_ping_t
{

};

struct COMM_PACKED sbx_msg_pong_t
{
    sbx_uptime_t uptime;
};

struct COMM_PACKED sbx_msg_t
{
    sbx_msg_type type;
    union COMM_PACKED {
        sbx_msg_ping_t ping;

        sbx_msg_hello_t hello;
        sbx_msg_pong_t pong;
        sbx_msg_status_t status;
        sbx_msg_sensor_stream_t sensor_stream;
        sbx_msg_ds18b20_t ds18b20;
        sbx_msg_noise_t noise;
        sbx_msg_light_t light;
    } payload;
};

namespace SENSOR_STREAM {

static constexpr std::size_t MAX_PAYLOAD_SIZE =
        ::MAX_XBEE_PAYLOAD_SIZE - sizeof(::sbx_msg_type);
static constexpr std::size_t MAX_ENCODED_SAMPLE_BYTES =
        (MAX_PAYLOAD_SIZE - sizeof(::sbx_msg_sensor_stream_t));
static constexpr std::size_t MAX_SAMPLES =
        ((8*MAX_ENCODED_SAMPLE_BYTES)/9);
static constexpr std::size_t MAX_BITMAP_SIZE = ((MAX_SAMPLES+7)/8);

}


#endif // COMM_SBX_H
