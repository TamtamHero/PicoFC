#include "crsf.h"

// C
#include <stdio.h>
#include <string.h>

// Pico SDK
#include "hardware/uart.h"

// PicoFC code
#include "../misc/crc8.h"

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

crsf_frame_t cur_frame;
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
                memcpy(cur_frame.payload+cur_frame.payload_len-remaining_bytes, buffer, bytes_read);
            }
            break;

            case CRSF_STATE_CRC: {
                // check CRC8 DVB-S2
                uint8_t crc = crc8_update(0, &cur_frame.type, 1);
                crc = crc8_update(crc, cur_frame.payload, cur_frame.payload_len);
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
                printf("Error: parser entered unknown state\n");
                break;
        }

        // advance read offsets
        len -= bytes_read;
        cur_frame.received += bytes_read;
        buffer += bytes_read;

        // if done with frame, prepare for next one
        if(cur_frame.state == CRSF_STATE_FINALIZED){
            memset(&cur_frame, 0, sizeof(cur_frame));
            printf("***\n");
        }
        else if(cur_frame.state == CRSF_STATE_ERROR){
            // printf("reset frame\n");
            memset(&cur_frame, 0, sizeof(cur_frame));
            printf("!!!\n");
        }
    }
}

uint8_t buffer[100];
size_t rx = 0;
// RX interrupt handler
void on_uart_rx() {

    while (uart_is_readable(UART_ID)) {
        buffer[rx++] = uart_getc(UART_ID);
    }
    if(rx >= 64){
        crsf_parser(buffer, rx);
        rx = 0;
    }
}