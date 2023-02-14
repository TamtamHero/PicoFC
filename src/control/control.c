#include "control.h"

// C
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// Pico SDK
#include "pico/platform.h"

// PicoFC code
#include "../context.h"

/*Scale pwm values from 988us - 2012us to angle values [50.0° : -50.0°]
988us -> 50.0
2012us -> -50.0

angle(pwm) = a.pwm + b
a = (-50.0-50.0)/(2012-988) =  -0.09765625
b = -50.0 - (a * 2012) = 146.484375
*/
float pwm_to_angle(uint16_t pwm){
    return (float)(COEFF_ANGLE_A * pwm + COEFF_ANGLE_B);
}

#define PITCH 0
#define ROLL 1
#define YAW 2
#define P_PITCH 12 // 12
#define I_PITCH 0.03
#define D_PITCH 1.4 // 1.4
#define P_ROLL P_PITCH
#define I_ROLL I_PITCH
#define D_ROLL D_PITCH
#define P_YAW 2
#define I_YAW 12
#define D_YAW 0
float P[3] = {P_PITCH, P_ROLL, P_YAW};
float I[3] = {I_PITCH, I_ROLL, I_YAW};
float D[3] = {D_PITCH, D_ROLL, D_YAW};
float angle_error_integral[3] = {0}; // used for I
float prev_angle_error[3] = {0}; // used to compute slope for D and to update angle_error_integral
void update_pid(uint16_t motor_target[NUM_MOTORS], float time_interval){
    float angle_error[3] = {
        picoFC_ctx.command.pitch - picoFC_ctx.orientation.ang_x,
        picoFC_ctx.command.roll - picoFC_ctx.orientation.ang_y,
        picoFC_ctx.command.yaw - picoFC_ctx.orientation.ang_z,
    };

    float pid_offset[3];
    // for each euler angle
    for (size_t i = 0; i < 3; i++){
        angle_error_integral[i] += (prev_angle_error[i] + angle_error[i])*time_interval/2;
        angle_error_integral[i] = angle_error_integral[i] > 200 ? 200 : angle_error_integral[i] < -200 ? -200 : angle_error_integral[i];
        float error_slope = (angle_error[i] - prev_angle_error[i])/time_interval;
        pid_offset[i] = P[i] * angle_error[i] + I[i] * angle_error_integral[i] + D[i] * error_slope;
    }

    // current error becomes previous error
    memcpy(prev_angle_error, angle_error, sizeof(prev_angle_error));

    // adapt motor speeds
    motor_target[0] = (uint16_t)(picoFC_ctx.command.throttle + pid_offset[PITCH] + pid_offset[ROLL]);// + pid_offset[YAW]);
    motor_target[1] = (uint16_t)(picoFC_ctx.command.throttle - pid_offset[PITCH] + pid_offset[ROLL]);// - pid_offset[YAW]);
    motor_target[2] = (uint16_t)(picoFC_ctx.command.throttle - pid_offset[PITCH] - pid_offset[ROLL]);// + pid_offset[YAW]);
    motor_target[3] = (uint16_t)(picoFC_ctx.command.throttle + pid_offset[PITCH] - pid_offset[ROLL]);// - pid_offset[YAW]);

    // make sure targets are valid DSHOT values
    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
        motor_target[i] = MIN(1250, motor_target[i]);
        motor_target[i] = MAX(DSHOT_MIN, motor_target[i]);
    }
}