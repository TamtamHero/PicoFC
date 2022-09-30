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
#include "hardware/pwm.h"

// PicoFC code
#include "../context.h"

/*Scale pwm values from 988us - 2012us to level for a 10kHz frequency
freq = pico_clock / wrap
level = wrap * duty
=>
level = pico_clock * duty / freq = duty * pwm_clock
eg: level = 125mHz * 25% / 10kHz = 3125
so level goes from 0 to 12500, and we need to map the input pwm value to that.
first, let's map duty:
988us -> 0
2012us -> 1
duty = 1/(2012-988)*pwm - 988/(2012-988) = pwm/1024 - 988/1024
then, level = 125mHz * duty / 10kHz = duty * pwm_clock
*/
uint16_t pwm_to_level(uint16_t pwm){
    return (uint16_t)(((float)pwm / 1024 - (float)988/1024) * 10000);
}

void task_leds(void* unused_arg){
    printf("=== hello from leds! \n");
    gpio_set_function(26, GPIO_FUNC_PWM);
    gpio_set_function(27, GPIO_FUNC_PWM);
    gpio_set_function(28, GPIO_FUNC_PWM);
    gpio_set_function(29, GPIO_FUNC_PWM);

    // Set period of 12500 cycles, so we get a frequency of 10kHz
    pwm_set_wrap(pwm_gpio_to_slice_num(26), 12500);
    pwm_set_wrap(pwm_gpio_to_slice_num(27), 12500);
    pwm_set_wrap(pwm_gpio_to_slice_num(28), 12500);
    pwm_set_wrap(pwm_gpio_to_slice_num(29), 12500);

    pwm_set_enabled(pwm_gpio_to_slice_num(26), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(27), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(28), true);
    pwm_set_enabled(pwm_gpio_to_slice_num(29), true);

    const TickType_t xDelay = 20 / portTICK_PERIOD_MS;

    while(true){
        // printf("leds: %d %d %d %d\n", picoFC_ctx.channels[0], picoFC_ctx.channels[1], picoFC_ctx.channels[2], picoFC_ctx.channels[3]);
        // printf("levels: %d %d %d %d\n", pwm_to_level(picoFC_ctx.channels[0]), pwm_to_level(picoFC_ctx.channels[1]), pwm_to_level(picoFC_ctx.channels[2]), pwm_to_level(picoFC_ctx.channels[3]));
        pwm_set_gpio_level(26, pwm_to_level(picoFC_ctx.channels[0]));
        pwm_set_gpio_level(27, pwm_to_level(picoFC_ctx.channels[1]));
        pwm_set_gpio_level(28, pwm_to_level(picoFC_ctx.channels[2]));
        pwm_set_gpio_level(29, pwm_to_level(picoFC_ctx.channels[3]));

        // sleep a bit until next update
        vTaskDelay(xDelay);
    }
}