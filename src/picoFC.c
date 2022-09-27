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

// 3rd party peripherals drivers
#include "mpu6050/driver_mpu6050.h"

// PicoFC code
#include "sensors/driver_mpu6050_interface.h"
#include "radio/crsf.h"

// By default these devices  are on bus address 0x68
static int addr = 0x68;

// This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// Record references to the tasks
TaskHandle_t mpu6050_task_handle = NULL;
TaskHandle_t crsf_task_handle = NULL;


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
