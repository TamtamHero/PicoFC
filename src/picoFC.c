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

// Peripherals drivers
#include "driver_mpu6050_interface.h"
#include "mpu6050/driver_mpu6050.h"

// By default these devices  are on bus address 0x68
static int addr = 0x68;

// This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// Record references to the tasks
TaskHandle_t mpu6050_task_handle = NULL;

void task_mpu6050(void* unused_arg){
    mpu6050_basic_init(addr);

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
    printf("Hello, MPU6050! Reading raw data from registers... %d %d\n", PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);

    // Store handles referencing the task; get return value
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t mpu6050_task_status = xTaskCreate(task_mpu6050,
                                         "MPU6050_TASK",
                                         512,
                                         NULL,
                                         1,
                                         &mpu6050_task_handle);

    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));

    // Start the FreeRTOS scheduler
    if (mpu6050_task_status == pdPASS) {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while(true) {
        // NOP
    };

    return 0;
}
