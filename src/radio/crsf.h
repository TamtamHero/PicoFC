#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define UART_ID uart1
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 8
#define UART_RX_PIN 9

#define CRSF_CHAN_COUNT 16
#define CRSF_SCALE_FACTOR 0.62477120195241
#define CRSF_SCALE_OFFSET 880.53935326418548

typedef enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
} crsf_addr_e;

typedef enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
} crsf_type_e;

typedef enum {
    CRSF_STATE_ADDR,
    CRSF_STATE_FRAME_LEN,
    CRSF_STATE_FRAME_TYPE,
    CRSF_STATE_PAYLOAD,
    CRSF_STATE_CRC,
    CRSF_STATE_FINALIZED,
    CRSF_STATE_ERROR,
} crsf_parser_state_e;

struct crsf_fc_payload_rc_channels_packed_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsf_fc_payload_rc_channels_packed_s crsf_fc_payload_rc_channels_packed_t;

union crsf_msg_u{
        crsf_fc_payload_rc_channels_packed_t rc_channels_packed;
};

typedef union crsf_msg_u crsf_msg_t;

struct crsf_frame_s{
    // this parser is a state machine
    crsf_parser_state_e state;
    crsf_addr_e address;
    // Stores frame's expected length (excluding address and length) once frame length byte has been processed, 0 otherwise
    size_t frame_len;
    crsf_type_e type;
    //Should be equal to `.frame_len` - 2
    size_t payload_len;
    union {
        // A CRSF payload can be 60 bytes long at most
        uint8_t raw[60];
        crsf_msg_t msg;
    } payload;
    // Number of frame bytes received (should be equal to `.frame_len` + 2 when finalized)
    size_t received;
};

typedef struct crsf_frame_s crsf_frame_t;

void on_uart_rx();

#endif