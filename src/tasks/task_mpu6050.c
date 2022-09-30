#include "tasks.h"

// C
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Pico SDK
#include "pico/stdlib.h"

// PicoFC code
#include "../sensors/driver_mpu6050_interface.h"

void task_mpu6050(void* unused_arg){
    mpu6050_basic_init(MPU6050_I2C_BASE_ADDR);
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
        // mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
        // mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
        // mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
        // mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
        // mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
        // mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
        // mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", degrees);

        sleep_ms(1000);
        count++;
    }
}