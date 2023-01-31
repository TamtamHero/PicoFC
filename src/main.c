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

// PicoFC code
#include "tasks/tasks.h"
#include "context.h"

// This is the inter-task queue
volatile QueueHandle_t queue = NULL;

// Record references to the tasks
TaskHandle_t mpu6050_task_handle = NULL;
TaskHandle_t crsf_task_handle = NULL;
// TaskHandle_t leds_task_handle = NULL;
TaskHandle_t motors_task_handle = NULL;

// PicoFC runtime context
picoFC_ctx_t picoFC_ctx;

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

    // BaseType_t crsf_task_status = xTaskCreate(task_crsf,
    //                                      "CRSF_TASK",
    //                                      16384,
    //                                      NULL,
    //                                      1,
    //                                      &crsf_task_handle);

    // BaseType_t motors_task_status = xTaskCreate(task_motors,
    //                                      "MOTORS_TASK",
    //                                      16384,
    //                                      NULL,
    //                                      1,
    //                                      &motors_task_handle);

    // BaseType_t leds_task_status = xTaskCreate(task_leds,
    //                                      "LEDS_TASK",
    //                                      16384,
    //                                      NULL,
    //                                      1,
    //                                      &leds_task_handle);

    // Set up the event queue
    queue = xQueueCreate(4, sizeof(uint8_t));

    // Start the FreeRTOS scheduler
    if (mpu6050_task_status == pdPASS) { // && crsf_task_status == pdPASS && motors_task_status == pdPASS) {
        vTaskStartScheduler();
    }

    // We should never get here, but just in case...
    while(true) {
        // NOP
    };

    return 0;
}
