#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../../src/pid.h"
#include "../../../../lib/icm20948/src/pico-icm20948.h"
#include "../../../../lib/icm20948/MadgwickAHRS/MadgwickAHRS.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// icm-20948
const uint i2c1_sda = 14;
const uint i2c1_scl = 15;
//int16_t mag_avg_first[3] = {0};

icm20948_config_t config = {0x68, 0x0C, i2c1};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.1f, {1.0f, 0.0f, 0.0f, 0.0f}};

// motor
static uint8_t slice_left;
static uint8_t slice_right;

// icm-20948
void icm20948_calc_data(float accel_g[3], float gyro_dps[3], float mag_ut[3]) {
    accel_g[0] = -(float)data.accel_raw[1] / 16384.0f; // センサー座標でx
    accel_g[1] = (float)data.accel_raw[0] / 16384.0f; // センサー座標でy
    accel_g[2] = (float)data.accel_raw[2] / 16384.0f; // センサー座標でz
    gyro_dps[0] = -(float)data.gyro_raw[1] / 131.0f; // roll
    gyro_dps[1] = (float)data.gyro_raw[0] / 131.0f; // pitch
    gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f; // yaw
    //mag_ut[0] = (float)data.mag_raw[1];
    //mag_ut[1] = (float)-data.mag_raw[0];
    //mag_ut[2] = (float)-data.mag_raw[2];
}

void q2e(madgwick_ahrs_t *data, float euler[]) {
    // 0: roll, 1: pitch, 2: yaw
    euler[0] = -1.0f * asinf(2.0f * (data->q[1]) * (data->q[3]) + 2.0f * (data->q[0]) * (data->q[2]));
    euler[1] = atan2f(2.0f * (data->q[2]) * (data->q[3]) - 2.0f * (data->q[0]) * (data->q[1]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[3]) * (data->q[3]) - 1.0f);
    euler[2] = atan2f(2.0f * (data->q[1]) * (data->q[2]) - 2.0f * (data->q[0]) * (data->q[3]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[1]) * (data->q[1]) - 1.0f);
    return;
}

#if 0
void icm20948_read_cal_mag_update(icm20948_config_t *config, int16_t *mag[3], int16_t mag_bias[3]) {
    icm20948_read_raw_mag(config, mag);
    static int16_t d0, d1, d2, f = 0, r = 1;
    d0 = mag[0] - mag_bias[0];
    d1 = mag[1] - mag_bias[1];
    d2 = mag[2] - mag_bias[2];
    f = d0*d0 + d1*d1 + d2*d2 - r*r;
    mag_bias[0] = mag_bias[0] + 4 * lr * f * d0;
    mag_bias[1] = mag_bias[1] + 4 * lr * f * d1;
    mag_bias[2] = mag_bias[2] + 4 * lr * f * d2;
    r = r + 4 * lr * f * r;
}
#endif

void icm20948_read_all(/* icm20948_config_t *config, icm20984_data_t *data */) {
    icm20948_read_cal_accel(&config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&config, &data.gyro_raw[0], &data.gyro_bias[0]);
    icm20948_read_cal_mag(&config, &data.mag_raw[0], &data.mag_bias[0]);
    //icm20948_read_temp_c(&config, &data.temp_c);
}

// motor
int16_t limit_pwm(int16_t pwm) {
    pwm = constrain(pwm, -1023, 1023);
    return pwm;
}

// all
bool update_all(repeating_timer_t *rt) {
    static float accel_g[3] = {0}, gyro_dps[3] = {0}, mag_ut[3] = {0}, euler[3] = {0};
    static float gyro_yawmag = 0;
    static float error[2] = {0}, integral = 0;
    static uint8_t times = 0;
    //static bool is_first = true;
    const float th = 0.4;
    float yawmag, mx, my;

    icm20948_read_all();
    icm20948_calc_data(accel_g, gyro_dps, mag_ut);
    MadgwickAHRSupdateIMU(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8);
    q2e(&filter, euler);

    #if 0
    if (is_first) {
        is_first = false;
        gyro_yawmag = -atan2f(mag_avg_first[0], mag_avg_first[1]);
        printf("gyro_yawmag: %3.2f\n", gyro_yawmag);
    }
    #endif
    
    if (times == 10) {
        times = 0;
        // 地磁気をセンサー座標系に合わせる 
        int16_t tmp = data.mag_raw[0];
        data.mag_raw[0] = data.mag_raw[1]; // x
        data.mag_raw[1] = tmp;             // y
        data.mag_raw[2] = -data.mag_raw[2];// z
        //yawmag = atanf(-(cos(euler[0])*((float)-data.mag_raw[0]/*y*/) - sin(euler[0])*((float)-data.mag_raw[2]/*z*/)) / (cos(euler[1])*((float)data.mag_raw[1]/*x*/) + sin(euler[1])*sin(euler[0])*((float)-data.mag_raw[0]/*y*/) + sin(euler[1])*cos(euler[0])*((float)-data.mag_raw[2]/*z*/)));
        mx = ((float)data.mag_raw[0])*cos(euler[1]) + ((float)data.mag_raw[1])*sin(euler[0])*sin(euler[1]) + ((float)data.mag_raw[2])*cos(euler[0])*sin(euler[1]);
        my = ((float)data.mag_raw[1])*cos(euler[0]) - ((float)data.mag_raw[2])*sin(euler[0]);
        //yawmag = atan2f((float)data.mag_raw[0], (float)data.mag_raw[1]);
        yawmag = -atan2f(mx, my);
        //yawmag = ((yawmag > 0) ? (M_PI - yawmag) : (M_PI + yawmag));
        int16_t pwm = (int16_t)pid_calc(yawmag, 0.0, error, &integral);
        //printf("D %3.2f %3.2f %3.2f\n", euler[0], euler[1], euler[2]);
        //printf("roll %3.2f pitch %3.2f yaw(6axis) %3.2f yaw(mag) %3.2f\n", euler[0], euler[1], euler[2], yawmag);
        //float yawmag_c = euler[2] + gyro_yawmag;
        //printf("pwm %4d e[2] %3.2f yaw %3.2f gy %3.2f i %f\n", pwm, euler[2], yawmag, yawmag*0.9 + yawmag_c*0.1, integral);
        printf("pwm %4d e[2] %3.2f yaw %3.2f i %f\n", pwm, euler[2], yawmag, integral);
        pwm = ((pwm>0)?(pwm+500):((pwm<0)?(pwm-500):0));
        pwm = limit_pwm(pwm);
        if ((yawmag > -th) && (yawmag < th)) {
            //printf("going straight!\n");
            motor_rotate(slice_left, 1023);
            motor_rotate(slice_right, 1023);
        } else if (pwm > 0) {
            // 右旋回
            motor_rotate(slice_left, pwm);
            //motor_rotate(slice_right, -pwm);
            motor_rotate(slice_right, (pwm -(pwm * 0.5)));
            //motor_rotate(slice_right, 0);
        } else if (pwm < 0) {
            // 左旋回
            //printf("left!\n");
            // pwm < 0 なので、abs(pwm) = -pwm
            //motor_rotate(slice_left, pwm);
            pwm = -pwm;
            motor_rotate(slice_left, (pwm -(pwm * 0.5)));
            //motor_rotate(slice_left, 0);
            motor_rotate(slice_right, pwm);
        }
        // ここらへん良い感じに左右旋回するようにあとで書く
        /*
        int16_t pwml = (int16_t)pid_calc((float)(-delta[0]), target_rotate, error_left, &integral_left);
        int16_t pwmr = (int16_t)pid_calc((float)(delta[1]), target_rotate, error_right, &integral_right);
        motor_rotate(slice_left, pwml);
        motor_rotate(slice_right, pwmr);
        */
    }
    times++;

    return true;
}

int main(void) {
    stdio_init_all();

    sleep_ms(2000);
    printf("\n\nhello, this is pico!\n");

    // motor
    slice_left = motor_init(20);
    slice_right = motor_init(18);
    pid_init(0.01, 300.0, 0.0, 100.0);

    // icm-20948
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl, GPIO_FUNC_I2C);

    printf("begin calibration. keep at rest\n");
    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");
    icm20948_cal_gyro(&config, &data.gyro_bias[0]);
    printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);
    icm20948_cal_accel(&config, &data.accel_bias[0]);
    printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);
    //data.mag_bias[0] = -102; data.mag_bias[1] = 38; data.mag_bias[2] = 146;
    data.mag_bias[0] = 0; data.mag_bias[1] = 634; data.mag_bias[2] = -329;

    #if 0
    for (uint8_t i = 0; i < 100; i++) {
        icm20948_read_cal_mag(&config, &data.mag_raw[0], &data.mag_bias[0]);
        mag_avg_first[0] += data.mag_raw[0];
        mag_avg_first[1] += data.mag_raw[1];
        mag_avg_first[2] += data.mag_raw[2];
        printf("mag_avg_first: %d %d %d\n", mag_avg_first[0], mag_avg_first[1], mag_avg_first[2]);
        sleep_ms(10);
    }
    mag_avg_first[0] /= 100;
    mag_avg_first[1] /= 100;
    mag_avg_first[2] /= 100;
    // 地磁気をセンサー座標系に合わせる 
    int16_t tmp = mag_avg_first[0];
    mag_avg_first[0] = mag_avg_first[1]; // x
    mag_avg_first[1] = tmp;             // y
    mag_avg_first[2] = -mag_avg_first[2];// z
    printf("mag_avg_first: %d %d %d\n", mag_avg_first[0], mag_avg_first[1], mag_avg_first[2]);
    #endif
    
    printf("calibration done\n");

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &update_all, NULL, &timer);

    while(1) tight_loop_contents();

    return 0;
}