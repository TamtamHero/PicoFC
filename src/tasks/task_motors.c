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
#include "../esc/esc.h"
#include "../control/control.h"

void parse_channels(){
    bool reset_request = picoFC_ctx.channels[4] > PWM_MIN;
    uint16_t throttle = pwm_to_dshot(picoFC_ctx.channels[1]);
    float pitch = pwm_to_angle(picoFC_ctx.channels[2]);
    float roll = pwm_to_angle(picoFC_ctx.channels[0]);
    float yaw = pwm_to_angle(picoFC_ctx.channels[3]);

    if(reset_request){
        printf("+++ reset requested +++\n");
    } else if(throttle < DSHOT_MIN || throttle > DSHOT_MAX){
        printf("+++ invalid dshot value for throttle +++\n");
    } else{
        picoFC_ctx.command.throttle = throttle;
        picoFC_ctx.command.pitch = pitch;
        picoFC_ctx.command.roll = roll;
        picoFC_ctx.command.yaw = yaw;
        return;
    }

    // in case of error, trigger reset watchdog and set throttle to 0;
    watchdog_enable(1, 1);
    // picoFC_ctx.command.throttle = DSHOT_MIN; // dangerous !
    return;
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
    sleep_ms(3000);

    // sleep 1ms between each update to motor speed
    const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

    uint32_t frame;
    uint16_t count = 0;
    uint16_t motor_target[NUM_MOTORS];
    uint64_t prev_time = time_us_64();
    while(true){
        // hardcode channels for tests:
        // picoFC_ctx.channels[1] = 1225; // ~25% throttle
        picoFC_ctx.channels[0] = 1500; // 0°
        picoFC_ctx.channels[2] = 1500;
        picoFC_ctx.channels[3] = 1500;
        // picoFC_ctx.channels[4] = PWM_MIN;

        parse_channels();

        uint64_t current_time = time_us_64();
        update_pid(motor_target, ((float)(current_time-prev_time) / 1000000.0));
        prev_time = current_time;

        // every 0.25s
        if(count > 250){
            count = 0;
            // printf("throttle: dshot:%d pwm:%d\n", picoFC_ctx.command.throttle, picoFC_ctx.channels[1]);
            printf("%u %u\n%u %u      /  x:%f y:%f z:%f\n", motor_target[3], motor_target[2], motor_target[0], motor_target[1],
            picoFC_ctx.orientation.ang_x, picoFC_ctx.orientation.ang_y, picoFC_ctx.orientation.ang_z);
        }

        for (int i = 0; i < NUM_MOTORS; ++i) {
            frame = (uint32_t)get_dshot_frame(motor_target[i]);
            pio_sm_put_blocking(pio0, sm+i, frame << 16u);
        }

        // sleep a bit until next update
        vTaskDelay(xDelay);
        count++;
    }
}