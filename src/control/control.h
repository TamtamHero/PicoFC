
#ifndef CONTROL_H
#define CONTROL_H

#include "stdint.h"

#include "../esc/esc.h"

struct command_s {
    uint16_t throttle; // dshot value
    float pitch; // degree
    float roll; // degree
    float yaw; // degree
};

typedef struct command_s command_t;

struct orientation_s {
    float ang_x; // pitch
    float ang_y; // roll
    float ang_z; // yaw
};

typedef struct orientation_s orientation_t;

#define ANGLE_MIN              (50.0)
#define ANGLE_MAX              (-50.0)
#define COEFF_ANGLE_A ((float)(ANGLE_MAX-ANGLE_MIN)/(PWM_MAX-PWM_MIN))
#define COEFF_ANGLE_B (ANGLE_MAX - (COEFF_ANGLE_A * PWM_MAX))

float pwm_to_angle(uint16_t pwm);
void update_pid(uint16_t motor_target[NUM_MOTORS], float time_interval);

#endif