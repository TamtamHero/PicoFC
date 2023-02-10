#include "esc.h"

uint8_t motor_pins[NUM_MOTORS] = {15, 26, 27, 28};

/*Scale pwm values from 988us - 2012us to dshot values [48 - 2047]
988us -> 48
2012us -> 2047

dshot(pwm) = a.pwm + b
a = (2047-48)/(2012-988) ~= 1.952
b = 2047 - (a * 2012) ~= -1880
*/
uint16_t pwm_to_dshot(uint16_t pwm){
    return (uint16_t)(COEFF_PWM_A * pwm + COEFF_PWM_B);
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