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
#define GYRO_COMPLEMENTARY_COEFF 0.98
#define EARTH_G 1.0
#define RAD_TO_DEG (180/M_PI)

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

// set to 1 every time the accelerometer's measurement are used to rectify pitch and roll
uint8_t flag_az_rectified = 0;


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

/* return True if value is equal to target ±tolerance
if percentage is True, then tolerance is interpreted as percentage, not as an absolute value */
bool in_boundaries(float target, float value, float tolerance, bool percentage){
    if(percentage){
        return value >= target*(1.0 - tolerance/100.0) && value <= target*(1.0 + tolerance/100.0) ? true : false;
    }
    else{
        return value <= target+tolerance && value >= target-tolerance ? true : false;
    }
}

/* return an angle in [-pi°, pi°]
*/
float normalize_angle(float angle){
    angle = fmod(angle, 360.0);
    if(angle < -180.0){
        angle += 360.0;
    }
    else if(angle > 180){
        angle  -= 360.0;
    }
    return angle;
}

float angle_weighted_average(float a1, float a2, float coeff_a1){
    a1 = normalize_angle(a1);
    a2 = normalize_angle(a2);
    if((a1 < 0) ^ (a2 < 0) && fabs(a1) + fabs(a2) > 180.0){
        if(a1 < 0)
            a1 += 360.0;
        if(a2 < 0)
            a2 += 360.0;
    }
    return normalize_angle(a1*coeff_a1+a2*(1.0-coeff_a1));
}

void init_orientation_tracking(){
    start_time = time_us_64();
    prev_time = start_time;
}

#define AX g[0]
#define AY g[1]
#define AZ g[2]
#define AX_N (AX/acc_total)
#define AY_N (AY/acc_total)
#define AZ_N (AZ/acc_total)
uint lolcount = 0;
void update_orientation(float g[3], float dps[3]){
    uint64_t cur_time = time_us_64();
    uint64_t time_increment = cur_time - prev_time;
    if(time_increment > 0){
        // gyroscope data
        cur_orientation.ang_x = normalize_angle(cur_orientation.ang_x + compute_orientation_delta(time_increment, dps[0]));
        cur_orientation.ang_y = normalize_angle(cur_orientation.ang_y + compute_orientation_delta(time_increment, -dps[1]));
        cur_orientation.ang_z = normalize_angle(cur_orientation.ang_z + compute_orientation_delta(time_increment, dps[2]));
        // cur_orientation.ang_z = 0;

        // use accelerometer data only when drone experience a total acceleration of 1g ±1%
        float acc_total = sqrt(pow(AX, 2) + pow(AY, 2) + pow(AZ, 2));
        flag_az_rectified = 0;
        if(in_boundaries(EARTH_G, acc_total, 1.0, true)){
            // float acc_ang_x = 90.0 - acos(AY/acc_total)*RAD_TO_DEG;
            // float acc_ang_y = 90.0 - acos(AX/acc_total)*RAD_TO_DEG;
            // float acc_ang_x = atan2(AY_N,(sqrt(pow(AX_N, 2) + pow(AZ_N, 2))))*RAD_TO_DEG;
            // float acc_ang_y = atan(AX_N/(sqrt(pow(AY_N, 2) + pow(AZ_N, 2))))*RAD_TO_DEG;
            // float z_sign = AZ >= 0 ? 1 : -1;
            // float acc_ang_x = atan2(AY,z_sign*sqrt(pow(AZ, 2)+pow(AX, 2)*0.01))*RAD_TO_DEG;
            // float acc_ang_y = atan(AX/sqrt(pow(AY, 2) + pow(AZ, 2)))*RAD_TO_DEG;

            if(AZ > 0.5){
                float acc_ang_x = atan2(AY,AZ)*RAD_TO_DEG;
                float acc_ang_y = atan(AX/sqrt(pow(AY, 2) + pow(AZ, 2)))*RAD_TO_DEG;
                cur_orientation.ang_x = angle_weighted_average(cur_orientation.ang_x, acc_ang_x, GYRO_COMPLEMENTARY_COEFF);
                cur_orientation.ang_y = angle_weighted_average(cur_orientation.ang_y, acc_ang_y, GYRO_COMPLEMENTARY_COEFF);
                flag_az_rectified = 1;
            }
        }

        prev_time = cur_time;
    }
}

void sense(float g[3], float dps[3], float m_gauss[3], float* temp){
    mpu6050_basic_read(g, dps);
    if(mpu6050_basic_read_temperature(temp)){
        printf("++++ ERROR DISCONNECTED MPU6050\n");
    }
    // hmc5883l_basic_read(m_gauss);

    // remove sensor error
    for(uint i=0; i<3; i++){
        g[i] = g[i] - offsets.g[i];
        dps[i] = dps[i] - offsets.dps[i];
        m_gauss[i] = m_gauss[i] - offsets.m_gauss[i];
    }
}

void task_mpu6050(void* unused_arg){
    mpu6050_basic_init(MPU6050_I2C_BASE_ADDR);
    sleep_ms(1000);
    // hmc5883l_basic_init();
    // sleep_ms(1000);
    init_orientation_tracking();

    float g[3];
    float dps[3];
    float m_gauss[3];
    float temp;
    uint count = 0;
    measure_idle_error();
    while (1) {

        sense(g, dps, m_gauss, &temp);
        update_orientation(g, dps);

        /* output */
        // mpu6050_interface_debug_print("mpu6050: %d.\n", count + 1);
        // mpu6050_interface_debug_print("mpu6050: acc x is %0.2fg.\n", g[0]);
        // mpu6050_interface_debug_print("mpu6050: acc y is %0.2fg.\n", g[1]);
        // mpu6050_interface_debug_print("mpu6050: acc z is %0.2fg.\n", g[2]);
        // mpu6050_interface_debug_print("mpu6050: gyro x is %0.2fdps.\n", dps[0]);
        // mpu6050_interface_debug_print("mpu6050: gyro y is %0.2fdps.\n", dps[1]);
        // mpu6050_interface_debug_print("mpu6050: gyro z is %0.2fdps.\n", dps[2]);
        // mpu6050_interface_debug_print("mpu6050: temperature %0.2fC.\n", temp);

        uint new_count = (prev_time - start_time)/25000; // count increases 40 times per second
        if(new_count > count){
            count = new_count;
            printf("{\"T\":%.2f,\"x\":%f,\"y\":%f,\"z\":%f,\"ax\":%f,\"ay\":%f,\"az\":%f,\"mx\":%.2f,\"my\":%.2f,\"mz\":%.2f,\"rec\":%s}\n"
            , temp
            , cur_orientation.ang_x, cur_orientation.ang_y, cur_orientation.ang_z
            , g[0], g[1], g[2]
            , m_gauss[0], m_gauss[1], m_gauss[2]
            , flag_az_rectified ? "\"+++\"" : "\"\"");
        }
    }
}