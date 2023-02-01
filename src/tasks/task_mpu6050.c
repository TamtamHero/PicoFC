#include "tasks.h"

// C
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

// Pico SDK
#include "pico/stdlib.h"

// PicoFC code
#include "../sensors/driver_mpu6050_interface.h"
#include "../sensors/driver_hmc5883l_interface.h"

#define ERROR_MEASUREMENT_ROUNDS 1000

struct mpu6050_error_s {
    float g[3];
    float dps[3];
    float m_gauss[3];
};

typedef struct mpu6050_error_s mpu6050_error_t;

mpu6050_error_t offsets;

struct orientation_s {
    float ang_x; // pitch
    float ang_y; // roll
    float ang_z; // yaw
};

typedef struct orientation_s orientation_t;

orientation_t cur_orientation = {0};

uint64_t start_time;
uint64_t prev_time;

// Measure average 0 error when device is idle so we can offset it afterward
void measure_idle_error(){
    float g[3];
    float dps[3];
    for(size_t i=0; i<ERROR_MEASUREMENT_ROUNDS; i++){
        mpu6050_basic_read(g, dps);
        for(size_t j=0; j<3; j++){
            offsets.dps[j] += dps[j]/ERROR_MEASUREMENT_ROUNDS;
        }
    sleep_ms(1);
    }
    // constant error approximately measured for accelerometer
    offsets.g[0] = 0.02;
    offsets.g[1] = 0;
    offsets.g[2] = 0.18;
    // constant error approximately measured for magnetometer
    offsets.m_gauss[0] = 0;
    offsets.m_gauss[1] = 0;
    offsets.m_gauss[2] = 0;
    printf("MPU6050 avg err: acc.x=%fg,acc.y=%fg,acc.z=%fg - gyro.x=%fdps,gyro.y=%fdps,gyro.z=%fdps - mag.x=%f,mag.y=%f,mag.z=%f\n"
    , offsets.g[0], offsets.g[1], offsets.g[2], offsets.dps[0], offsets.dps[1], offsets.dps[2], offsets.m_gauss[0], offsets.m_gauss[1], offsets.m_gauss[2]);
}

float compute_orientation_delta(uint64_t dt, float value){
    return ((float)dt / 1000000.0)*value; // dt in microseconds
}

void init_orientation_tracking(){
    start_time = time_us_64();
    prev_time = start_time;
}

void update_orientation(float g[3], float dps[3]){
    uint64_t cur_time = time_us_64();
    uint64_t time_increment = cur_time - prev_time;
    if(time_increment > 0){
        // gyroscope data
        cur_orientation.ang_x += compute_orientation_delta(time_increment, dps[0]);
        cur_orientation.ang_y += compute_orientation_delta(time_increment, -dps[1]);
        cur_orientation.ang_z += compute_orientation_delta(time_increment, dps[2]);

        // use accelerometer data only when drone experience a total acceleration of ~1g
        float acc_total = sqrt(pow(g[0], 2) + pow(g[1], 2) + pow(g[2], 2));
        if(acc_total > 0.999 && acc_total < 1.001){
            cur_orientation.ang_y = 90.0 - acos(g[0]/acc_total)*180/M_PI;
            cur_orientation.ang_x = 90.0 - acos(g[1]/acc_total)*180/M_PI;
        }

        prev_time = cur_time;
    }
}

void sense(float g[3], float dps[3], float m_gauss[3]){
    // mpu6050_basic_read(g, dps);
    hmc5883l_basic_read((float *)m_gauss);

    // remove sensor error
    for(uint i=0; i<3; i++){
        g[i] = g[i] - offsets.g[i];
        dps[i] = dps[i] - offsets.dps[i];
        m_gauss[i] = m_gauss[i] - offsets.m_gauss[i];
    }
}

void task_mpu6050(void* unused_arg){
    // mpu6050_basic_init(MPU6050_I2C_BASE_ADDR);
    // sleep_ms(1000);
    hmc5883l_basic_init();
    sleep_ms(1000);
    init_orientation_tracking();

    float g[3];
    float dps[3];
    float m_gauss[3];
    float degrees;
    uint count = 0;
    measure_idle_error();
    while (1) {

        sense(g, dps, m_gauss);
        update_orientation(g, dps);
        // mpu6050_basic_read_temperature(&degrees);
        if(degrees > 31){
            printf("++++ ERROR DISCONNECTED MPU6050\n");
        }

        /* output */
        // mpu6050_interface_debug_print("mpu6050: %d.\n", count + 1);
        // mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
        // mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
        // mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
        // mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
        // mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
        // mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
        // mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", degrees);

        uint new_count = (prev_time - start_time)/5000; // count increases every 0.05s
        if(new_count > count){
            count = new_count;
            printf("{\"x\":%f,\"y\":%f,\"z\":%f,\"ax\":%f,\"ay\":%f,\"az\":%f,\"mx\":%f,\"my\":%f,\"mz\":%f}\n"
            , cur_orientation.ang_x, cur_orientation.ang_y, cur_orientation.ang_z
            , g[0], g[1], g[2]
            , m_gauss[0], m_gauss[1], m_gauss[2]);
        }
    }
}