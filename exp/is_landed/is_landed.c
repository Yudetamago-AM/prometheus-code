#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../../lib/icm20948/src/pico-icm20948.h"


// icm-20948
icm20948_config_t config = {0x68, 0x0C, i2c1};
icm20984_data_t data;
const uint i2c1_sda = 14;
const uint i2c1_scl = 15;

int main(void) {
    stdio_init_all();
    printf("hello, this is \"is_landed!\"\n");

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(i2c1_sda, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl, GPIO_FUNC_I2C);

    printf("begin calibration of accel\n");
    if (icm20948_init(&config) == 0) printf("successfully initialized!\n");
    icm20948_cal_accel(&config, &data.accel_bias[0]);
    printf("calibrated accel: %d %d %d\n", data.accel_bias[0], data.accel_bias[1], data.accel_bias[2]);

    float abs_accel;
    float accel_g[3];
    while (1) {
        icm20948_read_cal_accel(&config, &data.accel_raw[0], &data.accel_bias[0]);
        accel_g[0] = -(float)data.accel_raw[1] / 16384.0f; // センサー座標でx
        accel_g[1] = (float)data.accel_raw[0] / 16384.0f; // センサー座標でy
        accel_g[2] = (float)data.accel_raw[2] / 16384.0f; // センサー座標でz
        //printf("0: %5.3f 1: %5.3f 2: %5.3f\n", accel_g[0], accel_g[1], accel_g[2]);
        abs_accel = (accel_g[0] * accel_g[0]) + (accel_g[1] * accel_g[1]) + (accel_g[2] * accel_g[2]);
        //abs_accel -= 1; // cancel gravity(easy approach)
        abs_accel = abs(abs_accel);
        //printf("before: %5.3f\n", abs_accel);
        abs_accel = sqrt(abs_accel);
        printf("abs_accel: %5.3f\n", abs_accel);
        sleep_ms(10);
    }

    return 0;
}