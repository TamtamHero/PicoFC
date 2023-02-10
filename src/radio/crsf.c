#include "crsf.h"

// C
#include <stdio.h>
#include <string.h>

// Pico SDK
#include "hardware/uart.h"

// PicoFC code
#include "../misc/misc.h"
#include "../context.h"

uint8_t rx_buffer[100];
size_t rx_count = 0;
crsf_frame_t cur_frame;

const uint8_t crsf_addr_table[] = {
    CRSF_ADDRESS_BROADCAST,
    CRSF_ADDRESS_USB,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO,
    CRSF_ADDRESS_RESERVED1,
    CRSF_ADDRESS_CURRENT_SENSOR,
    CRSF_ADDRESS_GPS,
    CRSF_ADDRESS_TBS_BLACKBOX,
    CRSF_ADDRESS_FLIGHT_CONTROLLER,
    CRSF_ADDRESS_RESERVED2,
    CRSF_ADDRESS_RACE_TAG,
    CRSF_ADDRESS_RADIO_TRANSMITTER,
    CRSF_ADDRESS_CRSF_RECEIVER,
    CRSF_ADDRESS_CRSF_TRANSMITTER
};

bool crsf_check_start_byte(uint8_t value){
    for(size_t i = 0; i < sizeof(crsf_addr_table)/sizeof(crsf_addr_table[0]); i++){
        if(value == crsf_addr_table[i]){
            return true;
        }
    }
    return false;
}

/* conversion from RC value to PWM
* for 0x16 RC frame
*       RC     PWM
* min  172 ->  988us
* mid  992 -> 1500us
* max 1811 -> 2012us
* scale factor = (2012-988) / (1811-172) = 0.62477120195241
* offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
*/
uint16_t rc_to_pwm(uint rc_value){
    return (uint16_t)((float)rc_value * CRSF_SCALE_FACTOR + CRSF_SCALE_OFFSET);
}

void crsf_handle_rc_channel_packed(crsf_fc_payload_rc_channels_packed_t* rc_channels_packed){
    /* we use this temporary buffer to avoid storing unscaled values in global context, even
    for a very short time. Could be avoided if we used a mutex to protect write i/o on context
    */
    uint16_t channels[CRSF_CHAN_COUNT];
    channels[0] = rc_channels_packed->chan0;
    channels[1] = rc_channels_packed->chan1;
    channels[2] = rc_channels_packed->chan2;
    channels[3] = rc_channels_packed->chan3;
    channels[4] = rc_channels_packed->chan4;
    channels[5] = rc_channels_packed->chan5;
    channels[6] = rc_channels_packed->chan6;
    channels[7] = rc_channels_packed->chan7;
    channels[8] = rc_channels_packed->chan8;
    channels[9] = rc_channels_packed->chan9;
    channels[10] = rc_channels_packed->chan10;
    channels[11] = rc_channels_packed->chan11;
    channels[12] = rc_channels_packed->chan12;
    channels[13] = rc_channels_packed->chan13;
    channels[14] = rc_channels_packed->chan14;
    channels[15] = rc_channels_packed->chan15;

    // scale rc values to pwm
    for(size_t i=0; i<CRSF_CHAN_COUNT; i++){
        picoFC_ctx.channels[i] = rc_to_pwm(channels[i]);
    }
}

void crsf_handle_message(crsf_addr_e addr, crsf_type_e type, crsf_msg_t* msg){
    switch (addr) {
    case CRSF_ADDRESS_FLIGHT_CONTROLLER: {
            switch (type) {
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
                    crsf_handle_rc_channel_packed(&msg->rc_channels_packed);
                }
                break;

            default:
                // printf("Error: payload type %02x is not supported\n", type);
                break;
            }
        }
        break;

    default:
        printf("Error: address %02x is not supported", addr);
        break;
    }
}

void crsf_parser(uint8_t* buffer, size_t len){
    // printf("=> %d state %d\n", len, cur_frame.state);
    if(len == 0){
        printf("Error: len can't be 0\n");
    }
    size_t bytes_read;
    while(len > 0){
        bytes_read = 1;
        switch (cur_frame.state) {
            case CRSF_STATE_ADDR: {
                uint8_t start_byte = *buffer;
                if(!crsf_check_start_byte(start_byte)){
                    // printf("bad start byte %02x %d\n", start_byte, len);
                    cur_frame.state = CRSF_STATE_ERROR;
                }
                else{
                    cur_frame.address = start_byte;
                    // printf("addr: %02x\n", cur_frame.address);
                    cur_frame.state = CRSF_STATE_FRAME_LEN;
                }
            }
            break;

            case CRSF_STATE_FRAME_LEN: {
                cur_frame.frame_len = *buffer;
                if(cur_frame.frame_len > 62){
                    printf("frame length too big: %d\n", cur_frame.frame_len);
                    cur_frame.state = CRSF_STATE_ERROR;
                }else{
                    cur_frame.payload_len = cur_frame.frame_len - 2;
                    // printf("frame length: %02x\n", cur_frame.frame_len);
                    cur_frame.state = CRSF_STATE_FRAME_TYPE;
                }
            }
            break;

            case CRSF_STATE_FRAME_TYPE: {
                cur_frame.type = *buffer;
                // printf("type: %02x\n", cur_frame.type);
                cur_frame.state = CRSF_STATE_PAYLOAD;
            }
            break;

            case CRSF_STATE_PAYLOAD: {
                // compute remaining payload bytes count
                uint8_t remaining_bytes = 3 + cur_frame.payload_len - cur_frame.received;
                if(len < remaining_bytes){
                    bytes_read = len;
                }
                else{
                    bytes_read = remaining_bytes;
                    // printf("payload complete\n");
                    cur_frame.state = CRSF_STATE_CRC;
                }
                memcpy(cur_frame.payload.raw+cur_frame.payload_len-remaining_bytes, buffer, bytes_read);
            }
            break;

            case CRSF_STATE_CRC: {
                // check CRC8 DVB-S2
                uint8_t crc = crc8_update(0, &cur_frame.type, 1);
                crc = crc8_update(crc, cur_frame.payload.raw, cur_frame.payload_len);
                uint8_t received_crc = *buffer;
                if(crc != received_crc){
                    printf("Error: wrong crc, 0x%02x received but 0x%02x computed\n", received_crc, crc);
                    cur_frame.state = CRSF_STATE_ERROR;
                }
                else{
                    // printf("crc checked\n");
                    cur_frame.state = CRSF_STATE_FINALIZED;
                }
            }
            break;

            default:
                printf("Error: parser entered unknown state %d\n", cur_frame.state);
                break;
        }

        // advance read offsets
        len -= bytes_read;
        cur_frame.received += bytes_read;
        buffer += bytes_read;

        // if done with frame, prepare for next one
        if(cur_frame.state == CRSF_STATE_FINALIZED){
            // printf("*");
            crsf_handle_message(cur_frame.address, cur_frame.type, &cur_frame.payload.msg);
            memset(&cur_frame, 0, sizeof(cur_frame));
        }
        else if(cur_frame.state == CRSF_STATE_ERROR){
            // printf("reset frame\n");
            memset(&cur_frame, 0, sizeof(cur_frame));
            printf("!!!\n");
        }
    }
}

// RX interrupt handler
void on_uart_rx() {

    while (uart_is_readable(UART_ID)) {
        rx_buffer[rx_count++] = uart_getc(UART_ID);
    }
    if(rx_count >= 64){
        crsf_parser(rx_buffer, rx_count);
        rx_count = 0;
    }
}