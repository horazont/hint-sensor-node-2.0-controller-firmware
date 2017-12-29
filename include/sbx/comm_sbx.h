#ifndef COMM_SBX_H
#define COMM_SBX_H

#include <stdint.h>

#ifdef XBEE_USE_ENCRYPTION
#define MAX_FRAME_PAYLOAD_SIZE (66)
#else
#define MAX_FRAME_PAYLOAD_SIZE (128)
#endif

#define SBX_LIGHT_SENSOR_SAMPLES (6)
#define SBX_LIGHT_SENSOR_CHANNELS (4)

#define COMM_PACKED __attribute__((packed))
#define CFFI_DOTDOTDOT
#define CFFI_DOTDOTDOT_or(x) x

#ifdef __cplusplus
#define NOT_IN_C(x) x
#define ONLY_IN_C(x)
#define COMM_ENUM_PACKED
#else
#define NOT_IN_C(x)
#define ONLY_IN_C(x) x
#define COMM_ENUM_PACKED COMM_PACKED
#endif

// I hate myself for doing this. But it helps CFFI.
#ifdef __cplusplus
#define enum enum class
#endif

#define SBX_BME280_DIG88_SIZE (26)
#define SBX_BME280_DIGE1_SIZE (7)
#define SBX_BME280_READOUT_SIZE (8)

enum COMM_ENUM_PACKED sbx_msg_type NOT_IN_C(:std::uint8_t) {
    PING = 0x01,

    HELLO = 0x80,
    PONG = 0x81,
    STATUS = 0x82,
    SENSOR_DS18B20 = 0xf1,
    SENSOR_NOISE = 0xf2,
    SENSOR_DHT = 0xf3,
    SENSOR_LIGHT = 0xf4,
    SENSOR_BME280 = 0xf5,
    SENSOR_STREAM_ACCEL_X = 0xf8,
    SENSOR_STREAM_ACCEL_Y = 0xf9,
    SENSOR_STREAM_ACCEL_Z = 0xfa,
    SENSOR_STREAM_COMPASS_X = 0xfb,
    SENSOR_STREAM_COMPASS_Y = 0xfc,
    SENSOR_STREAM_COMPASS_Z = 0xfd,
    RESERVED = 0xff,
};

#ifdef __cplusplus
#undef enum
#else
#define sbx_msg_type uint8_t
#endif


typedef uint16_t sbx_uptime_t;
typedef uint32_t sbx_rtc_t;

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

struct COMM_PACKED sbx_msg_ds18b20_sample_t
{
    /**
     * DS18B20 sensor ID
     */
    uint8_t id[8];

    /**
     * Raw 16 bit sensor value
     */
    uint16_t raw_value;
};

#define SBX_MAX_DS18B20_SAMPLES ((MAX_FRAME_PAYLOAD_SIZE-(sizeof(sbx_uptime_t)+sizeof(sbx_msg_type)))/sizeof(struct sbx_msg_ds18b20_sample_t))
#define SBX_NOISE_SAMPLES (16)

struct COMM_PACKED sbx_msg_ds18b20_t {
    /**
     * Milliseconds since boot.
     *
     * Wraps around every ~65 seconds or so. Since all sensors are sampled
     * more often than that, wraparound can be compensated for.
     */
    sbx_uptime_t timestamp;

    struct sbx_msg_ds18b20_sample_t samples[CFFI_DOTDOTDOT_or(SBX_MAX_DS18B20_SAMPLES)];
    CFFI_DOTDOTDOT
};

struct COMM_PACKED sbx_msg_noise_sample_t {
    /**
     * Timestamp of the sample.
     */
    sbx_uptime_t timestamp;

    /**
     * Raw 16 bit sensor value
     */
    uint16_t value;
};

struct COMM_PACKED sbx_msg_noise_t {
    struct sbx_msg_noise_sample_t samples[CFFI_DOTDOTDOT_or(SBX_NOISE_SAMPLES)];
    CFFI_DOTDOTDOT
};

struct COMM_PACKED sbx_msg_light_sample_t {
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

struct COMM_PACKED sbx_msg_light_t {
    /**
     * Array containing the samples.
     *
     * Six samples are safe even with encryption and provide a reasonable update
     * interval.
     */
    struct sbx_msg_light_sample_t samples[6];
};

struct COMM_PACKED sbx_msg_hello_t {
    sbx_rtc_t rtc;
    sbx_uptime_t uptime;
};

struct COMM_PACKED sbx_msg_status_t {
    sbx_rtc_t rtc;
    sbx_uptime_t uptime;

    /**
     * Core protocol version number.
     */
    uint8_t protocol_version;

    /**
     * Status version number.
     */
    uint8_t status_version;

    struct COMM_PACKED {
        /**
         * Current sequence numbers for the Accelerometer and Magnetometer sensor
         * streams.
         */
        struct COMM_PACKED {
            uint16_t sequence_number;
            uint16_t timestamp;
            uint16_t period;
        } stream_state[2];
    } imu;

    struct COMM_PACKED {
        uint16_t transaction_overruns;
    } i2c_metrics[2];

    struct COMM_PACKED {
        uint8_t configure_status;
        uint16_t timeouts;
    } bme280_metrics;
};

struct COMM_PACKED sbx_msg_dht11_t {
    sbx_uptime_t timestamp;
    uint16_t humidity;
    uint16_t temperature;
};

struct COMM_PACKED sbx_msg_bme280_t {
    sbx_uptime_t timestamp;
    uint8_t dig88[CFFI_DOTDOTDOT_or(SBX_BME280_DIG88_SIZE)];
    uint8_t dige1[CFFI_DOTDOTDOT_or(SBX_BME280_DIGE1_SIZE)];
    uint8_t readout[CFFI_DOTDOTDOT_or(SBX_BME280_READOUT_SIZE)];
    CFFI_DOTDOTDOT
};

union COMM_PACKED _sbx_msg_payload_t {
    struct sbx_msg_hello_t hello;
    struct sbx_msg_status_t status;
    struct sbx_msg_sensor_stream_t sensor_stream;
    struct sbx_msg_ds18b20_t ds18b20;
    struct sbx_msg_dht11_t dht11;
    struct sbx_msg_noise_t noise;
    struct sbx_msg_light_t light;
    struct sbx_msg_bme280_t bme280;
};

struct COMM_PACKED sbx_msg_t {
    NOT_IN_C(sbx_msg_type) ONLY_IN_C(uint8_t) type;
    union _sbx_msg_payload_t payload;
};



#ifdef __cplusplus
namespace SENSOR_STREAM {

static constexpr std::size_t MAX_PAYLOAD_SIZE =
        MAX_FRAME_PAYLOAD_SIZE - sizeof(::sbx_msg_type);
static constexpr std::size_t MAX_ENCODED_SAMPLE_BYTES =
        (MAX_PAYLOAD_SIZE - sizeof(::sbx_msg_sensor_stream_t));
static constexpr std::size_t MAX_SAMPLES =
        ((8*MAX_ENCODED_SAMPLE_BYTES)/9);
static constexpr std::size_t MAX_BITMAP_SIZE = ((MAX_SAMPLES+7)/8);

}

#undef CONSTEXPR
#endif

#ifndef __cplusplus
#undef sbx_msg_type
#endif


#endif // COMM_SBX_H
