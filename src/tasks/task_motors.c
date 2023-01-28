#include "tasks.h"

// FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

// C
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Pico SDK
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "dshot.pio.h"

// PicoFC code
#include "../context.h"

#define NUM_MOTORS 4
uint8_t motor_pins[NUM_MOTORS] = {15, 26, 27, 28};

#define PWM_MIN              (988)
#define PWM_MAX              (2012)
#define DSHOT_MIN              (48)
#define DSHOT_MAX              (2047)

#define COEFF_A ((float)(DSHOT_MAX-DSHOT_MIN)/(PWM_MAX-PWM_MIN))
#define COEFF_B (DSHOT_MAX - (COEFF_A * PWM_MAX))
/*Scale pwm values from 988us - 2012us to dshot values (48 - 2047)
988us -> 48
2012us -> 2047

dshot(pwm) = a.pwm + b
a = (2047-48)/(2012-988) ~= 1.952
b = 2047 - (a * 2012) ~= -1880
*/
uint16_t pwm_to_dshot(uint16_t pwm){
    return (uint16_t)(COEFF_A * pwm + COEFF_B);
}

uint16_t get_dshot_frame(uint16_t command){
    //compute the checksum. xor the three nibbles of the speed + the telemetry bit (not used here)
    uint16_t checksum = 0;
    uint16_t checksum_data = command << 1;
    for(uint8_t i=0; i < 3; i++) {
        checksum ^= checksum_data;
        checksum_data >>= 4;
    }
    checksum &= 0x000F; //we only use the least-significant four bits as checksum
    uint16_t dshot_frame = (command << 5) | checksum; //add in the checksum bits to the least-four-significant bits
    return dshot_frame;
}

void task_motors(void* unused_arg){
    printf("=== hello from motors! \n");

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &dshot_program);

    for(size_t i = 0; i<NUM_MOTORS; i++){
        dshot_program_init(pio, sm+i, offset, motor_pins[i], 600000);
    }

    // leave some time to get my hands out of the copter
    sleep_ms(5000);

    // sleep 1ms between each update to motor speed
    const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

    uint32_t frame;
    uint16_t power = DSHOT_MIN;
    uint16_t count = 0;
    bool error = false;
    while(!error){
        power = pwm_to_dshot(picoFC_ctx.channels[1]);
        if(picoFC_ctx.channels[4] > PWM_MIN){
            error = true;
            printf("+++ reset requested +++\n");
            watchdog_enable(1, 1);
            continue;
        } else if(picoFC_ctx.channels[1] < PWM_MIN || picoFC_ctx.channels[1] > PWM_MAX){
            error = true;
            printf("+++ invalid pwm value +++\n");
            watchdog_enable(1, 1);
            continue;
        }
        if(count > 1000){
            count = 0;
            printf("power: %d %d\n", power, picoFC_ctx.channels[1]);
        }
        frame = (uint32_t)get_dshot_frame(power);
        for (int i = 0; i < NUM_MOTORS; ++i) {
            pio_sm_put_blocking(pio0, sm+i, frame << 16u);
        }

        // sleep a bit until next update
        vTaskDelay(xDelay);
        count++;
    }
}