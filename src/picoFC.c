/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

// C
#include <stdio.h>
#include <string.h>

// Pico SDK
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

// Peripherals drivers
#include "driver_mpu6050_interface.h"
#include "mpu6050/driver_mpu6050.h"

#define UART_ID uart1
// #define BAUD_RATE 115200
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 8
#define UART_RX_PIN 9

// By default these devices  are on bus address 0x68
static int addr = 0x68;

// This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// Record references to the tasks
TaskHandle_t mpu6050_task_handle = NULL;
TaskHandle_t crsf_task_handle = NULL;

uint8_t crc8_calc(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t crc8_update(uint8_t crc, const void *data, size_t length)
{
    const uint8_t *p = (const uint8_t *)data;
    const uint8_t *pend = p + length;

    for (; p != pend; p++) {
        crc = crc8_calc(crc, *p);
    }
    return crc;
}

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

uint8_t crsf_addr_table[] = {
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

typedef struct{
    // this parser is a state machine
    crsf_parser_state_e state;
    crsf_addr_e address;
    // Stores frame's expected length (excluding address and length) once frame length byte has been processed, 0 otherwise
    size_t frame_len;
    crsf_type_e type;
    //Should be equal to `.frame_len` - 2
    size_t payload_len;
    // A CRSF payload can be 60 bytes long at most
    uint8_t payload[60];
    // Number of frame bytes received (should be equal to `.frame_len` + 2 when finalized)
    size_t received;
} crsf_frame_t;

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

void task_crsf(void* unused_arg){
    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    while (1)
        tight_loop_contents();
}

void task_mpu6050(void* unused_arg){
    mpu6050_basic_init(addr);
    sleep_ms(1000);

    float g[3];
    float dps[3];
    float degrees;
    uint count = 0;
    while (1) {

        mpu6050_basic_read(g, dps);
        mpu6050_basic_read_temperature(&degrees);

        /* output */
        mpu6050_interface_debug_print("mpu6050: %d.\n", count + 1);
        mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
        mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
        mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
        mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
        mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
        mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
        mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", degrees);

        sleep_ms(1000);
        count++;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("+++ Hello, PicoFC! +++\n");

    // Store handles referencing the task; get return value
    // NOTE Arg 3 is the stack depth -- in words (4 bytes)
    BaseType_t mpu6050_task_status = xTaskCreate(task_mpu6050,
                                         "MPU6050_TASK",
                                         2048,
                                         NULL,
                                         1,
                                         &mpu6050_task_handle);

    BaseType_t crsf_task_status = xTaskCreate(task_crsf,
                                         "CRSF_TASK",
                                         16384,
                                         NULL,
                                         1,
                                         &crsf_task_handle);

    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));

    // Start the FreeRTOS scheduler
    if (mpu6050_task_status == pdPASS && crsf_task_status == pdPASS) {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while(true) {
        // NOP
    };

    return 0;
}
