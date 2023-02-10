#ifndef ESC_H
#define ESC_H

#include "stdint.h"

#define NUM_MOTORS 4
extern uint8_t motor_pins[NUM_MOTORS];

#define PWM_MIN              (988)
#define PWM_MAX              (2012)
#define DSHOT_MIN              (48)
#define DSHOT_MAX              (2047)

#define COEFF_PWM_A ((float)(DSHOT_MAX-DSHOT_MIN)/(PWM_MAX-PWM_MIN))
#define COEFF_PWM_B (DSHOT_MAX - (COEFF_PWM_A * PWM_MAX))

uint16_t pwm_to_dshot(uint16_t pwm);
uint16_t get_dshot_frame(uint16_t command);

#endif