#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../lib/icm20948/src/pico-icm20948.h"
#include "../lib/icm20948/MadgwickAHRS/MadgwickAHRS.h"

#define I2C_SDA 14
#define I2C_SCL 15

#define ANGLE_TH 0.39//前45度，GPS誘導
#define LONGITUDE_PER_RES 111
#define LATITUDE_PER_RES 92

i2c_inst_t icm20948_i2c = {i2c1_hw, false};
icm20948_config_t config = {0x68, 0x0C, &icm20948_i2c};
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};
icm20984_data_t data;

void read_icm20948(repeating_timer_t *rt) {
    icm20948_read_cal_accel(&config, &data.accel_raw[0], &data.accel_bias[0]);
    icm20948_read_cal_gyro(&config, &data.gyro_raw[0], &data.gyro_bias[0]);
    icm20948_read_cal_mag(&config, &data.mag_raw[0], &data.mag_bias[0]);
    icm20948_read_temp_c(&config, &data.temp_c);
    accel_g[0] = (float)data.accel_raw[0] / 16384.0f;
    accel_g[1] = (float)data.accel_raw[1] / 16384.0f;
    accel_g[2] = (float)data.accel_raw[2] / 16384.0f;
    gyro_dps[0] = (float)data.gyro_raw[0] / 131.0f;
    gyro_dps[1] = (float)data.gyro_raw[1] / 131.0f;
    gyro_dps[2] = (float)data.gyro_raw[2] / 131.0f;
    mag_ut[0] = (float)data.mag_raw[1];
    mag_ut[1] = (float)-data.mag_raw[0];
    mag_ut[2] = (float)-data.mag_raw[2];
    MadgwickAHRSupdate(&filter, \
    ((float)data.gyro_raw[0] / 131.0f), ((float)data.gyro_raw[1] / 131.0f), ((float)data.gyro_raw[2] / 131.0f), \
    ((float)data.accel_raw[0] / 16384.0f) * 9.8, ((float)data.accel_raw[1] / 16384.0f) * 9.8, ((float)data.accel_raw[2] / 16384.0f) * 9.8, \
    ((float)data.mag_raw[1]), ((float)-data.mag_raw[0]), ((float)-data.mag_raw[2]));
}

void q2e(madgwick_ahrs_t *data, float euler[]) {
    // 0: roll, 1: pitch, 2: yaw
    euler[0] = -1.0f * asinf(2.0f * (data->q[1]) * (data->q[3]) + 2.0f * (data->q[0]) * (data->q[2]));
    euler[1] = atan2f(2.0f * (data->q[2]) * (data->q[3]) - 2.0f * (data->q[0]) * (data->q[1]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[3]) * (data->q[3]) - 1.0f);
    euler[2] = atan2f(2.0f * (data->q[1]) * (data->q[2]) - 2.0f * (data->q[0]) * (data->q[3]), 2.0f * (data->q[0]) * (data->q[0]) + 2.0f * (data->q[1]) * (data->q[1]) - 1.0f);
    return;
}

void calc_gps(double *direction, double *distance) {
    while (1) {
        if (/*新しいデータ*/) {
            int32_t dx = ((goal_longitude - //long) * LONGITUDE_PER_RES);//0.000001度で0.092m(京田辺)，0.085m(能代)より，単位メートル
            int32_t dy = ((goal_latitude - //lat) * LATITUDE_PER_RES);//0.000001度で0.111m(111)より0.1
            
            if (dx == 0 && dy == 0) *direction = 0;
            else *direction = atan2(dx, dy);//意図的にdx, dyの順，というのも，北基準だから．
            *distance = approx_distance(dx, dy) / 10;//単位:cm
        }
        
    }
    
}

/*平方根を使わず2点間の距離を近似*/
//参考
//https://nowokay.hatenablog.com/entry/20120604/1338773843
//https://dora.bk.tsukuba.ac.jp/~takeuchi/?%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%2F%E5%B9%B3%E6%96%B9%E6%A0%B9%E3%82%92%E4%BD%BF%E3%82%8F%E3%81%9A%E3%81%AB%E8%B7%9D%E9%9B%A2%E3%82%92%E6%B1%82%E3%82%81%E3%82%8B
int32_t approx_distance(int32_t dx, int32_t dy) {
   uint32_t min, max, approx;

   if (dx < 0) dx = -dx;
   if (dy < 0) dy = -dy;

   if (dx < dy) {
      min = dx;
      max = dy;
   } else {
      min = dy;
      max = dx;
   }

   approx = (max * 983) + (min * 407);
   if (max < (min << 4)) approx -= ( max * 40 );

   // add 512 for proper rounding
   return ((approx + 512) >> 10);
} 

int main(void) {
    stdio_init_all();

    i2c_init(&icm20948_i2c, 400 * 1000); // 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    sleep_ms(2000);
    printf("hello, this is pico!\n");
    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");
    icm20948_cal_gyro(&config, &data.gyro_bias[0]);
    printf("calibrated gyro: %d %d %d\n", data.gyro_bias[0], data.gyro_bias[1], data.gyro_bias[2]);
    icm20948_cal_accel(&config, &data.accel_bias[0]);
    printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);
// set mag_bias manually (add cal func later)
    data.mag_bias[0] = -102; data.mag_bias[1] = 38; data.mag_bias[2] = 146;
    sleep_ms(2000);
    printf("calibration done\n");

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &read_icm20948, NULL, &timer);

    while (1) {
        // GPSのデータ取得

        // 
    }

}