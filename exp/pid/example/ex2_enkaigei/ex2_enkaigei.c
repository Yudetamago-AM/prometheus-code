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

icm20948_config_t config = {0x68, 0x0C, i2c1};
icm20984_data_t data;
madgwick_ahrs_t filter = {0.3f, {1.0f, 0.0f, 0.0f, 0.0f}};

// motor
static uint8_t slice_left;
static uint8_t slice_right;

// icm-20948
void icm20948_calc_data(float accel_g[3], float gyro_dps[3], float mag_ut[3]) {
    accel_g[0] = (float)data.accel_raw[0] / 16384.0f;
    accel_g[1] = (float)data.accel_raw[1] / 16384.0f;
    accel_g[2] = (float)data.accel_raw[2] / 16384.0f;
    gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
    gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;
    gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f;
    mag_ut[0] = (float)data.mag_raw[1];
    mag_ut[1] = (float)-data.mag_raw[0];
    mag_ut[2] = (float)-data.mag_raw[2];
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
    static float error[2] = {0}, integral = 0;
    float th = 0.39;

    icm20948_read_all();
    icm20948_calc_data(accel_g, gyro_dps, mag_ut);
    MadgwickAHRSupdate(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8, mag_ut[0], mag_ut[1], mag_ut[2]);
    q2e(&filter, euler);

    int16_t pwm = (int16_t)pid_calc(euler[2], 0.0, error, &integral);
    printf("pwm %4d e[2] %3.2f i %f\n", pwm, euler[2], integral);
    pwm = ((pwm>0)?(pwm+500):((pwm<0)?(pwm-500):0));
    pwm = limit_pwm(pwm);
    if ((euler[2] > -th) && (euler[2] < th)) {
        printf("going straight!\n");
        motor_rotate(slice_left, 1023);
        motor_rotate(slice_right, 1023);
    } else if (pwm > 0) {
        // 右旋回
        motor_rotate(slice_left, pwm);
        //motor_rotate(slice_right, -pwm);
        motor_rotate(slice_right, 0);
    } else {
        // 左旋回
        // pwm < 0 なので、abs(pwm) = -pwm
        //motor_rotate(slice_left, pwm);
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, -pwm);
    }
    // ここらへん良い感じに左右旋回するようにあとで書く
    /*
    int16_t pwml = (int16_t)pid_calc((float)(-delta[0]), target_rotate, error_left, &integral_left);
    int16_t pwmr = (int16_t)pid_calc((float)(delta[1]), target_rotate, error_right, &integral_right);
    motor_rotate(slice_left, pwml);
    motor_rotate(slice_right, pwmr);
    */

    return true;
}

int main(void) {
    stdio_init_all();

    sleep_ms(2000);
    printf("\n\nhello, this is pico!\n");

    // motor
    slice_left = motor_init(20);
    slice_right = motor_init(18);
    pid_init(0.01, 300.0, 0.0, 0.0);

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
    data.mag_bias[0] = -102; data.mag_bias[1] = 38; data.mag_bias[2] = 146;
    printf("calibration done\n");

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &update_all, NULL, &timer);

    while(1) tight_loop_contents();

    return 0;
}