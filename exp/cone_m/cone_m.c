#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../../lib/vl53l5cx/api/vl53l5cx_api.h"
#include "../pid/src/pid.h"
//#include "../../lib/icm20948/src/pico-icm20948.h"
//#include "../../lib/icm20948/MadgwickAHRS/MadgwickAHRS.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


// vl53l5cx
const uint tof_sda = 8;
const uint tof_scl = 9;
const uint tof_vcc = 11; // LOW to power on

i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};

VL53L5CX_Configuration dev;
VL53L5CX_ResultsData res;

// motor
static uint8_t slice_left;
static uint8_t slice_right;

// motor
int16_t limit_pwm(int16_t pwm) {
    pwm = constrain(pwm, -1023, 1023);
    return pwm;
}

bool update_all(repeating_timer_t *rt) {
    uint8_t is_ready;
    static int8_t i = 0;
    static int16_t tof_mat[4][64];
    static int32_t history[20] = {4000};
    int8_t lflg = 0;

    vl53l5cx_check_data_ready(&dev, &is_ready);
    if (is_ready) {
        vl53l5cx_get_ranging_data(&dev, &res);

        //printf("data:\n");
        // qsortからの中央値だして比較？
        if (i == 2) {
            lflg = 0;
            i = 0;
            for (int j = 0; j < 64; j++) tof_mat[3][j] = (tof_mat[1][j] + tof_mat[2][j] + tof_mat[3][j]) / 3;

            for (int j = 0; j < 20-1; j++) history[j] = history[j+1];
            for (int j = 0; j < 64; j++) history[19] += tof_mat[3][j];
            history[19] /= 64;

            for (int8_t i = 0; i < 19; i++) {
                if (history[i] > history[19]) lflg++;
            }
            printf("h1: %d h2: %d h3: %d h18: %d\n", history[1], history[2], history[3], history[18]);
            printf("lflg: %d, h19: %d\n", lflg, history[19]);

            if (lflg > 18) {
                
                
                motor_rotate(slice_left, 1023);
                motor_rotate(slice_right, 1023);
                printf("staright!\n");
            } else {
                motor_rotate(slice_left, 800);
                motor_rotate(slice_right, 0);
                printf("rotate\n");
            }
        }

        for (int j = 0; j < 64; j++) tof_mat[i][j] = res.distance_mm[j];
        i++;
    }

    

    return true;
}


int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    printf("hello, this is \"cone_m\"\n");

    // motor
    slice_left = motor_init(20);
    slice_right = motor_init(18);
    pid_init(0.01, 300.0, 0.0, 100.0);

    // tof
    gpio_init(tof_vcc);
    gpio_set_dir(tof_vcc, GPIO_OUT);
    gpio_put(tof_vcc, 0);

    i2c_init(&vl53l5cx_i2c, 400 * 1000);
    gpio_set_function(tof_scl, GPIO_FUNC_I2C);
    gpio_set_function(tof_sda, GPIO_FUNC_I2C);

    int8_t status, is_alive;

    dev.platform.address = 0x29;
    dev.platform.i2c     = &vl53l5cx_i2c;

    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (!is_alive || status) printf("err: sensor not detected\n");

    status = vl53l5cx_init(&dev);
    if (status) printf("ULD load failed\n");

    // 8x8
    status = vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);
    if (status) printf("err: set_resolution\n");
    // 5Hz
    status = vl53l5cx_set_ranging_frequency_hz(&dev, 15);
    if (status) printf("err: set_ranging_frequency_hz\n");
    // strongest -> closest
    status = vl53l5cx_set_target_order(&dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    if (status) printf("err: set_target_order\n");
    // continuous
    status = vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    if (status) printf("err: set_ranging_mode\n");
    status = vl53l5cx_start_ranging(&dev);
    if (status) printf("err: start_ranging\n");

    printf("setup done!\n");

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &update_all, NULL, &timer);

    while (1) tight_loop_contents();
    return 0;

#if 0
    int8_t i = 0;
    while (1) {
        //if (getchar() == 'q') goto exit;
        status = vl53l5cx_check_data_ready(&dev, &is_ready);
        if (is_ready) {
            vl53l5cx_get_ranging_data(&dev, &res);

            //printf("data:\n");
            if (i == 2) {
                i = 0;
                for (int j = 0; j < 64; j++) tof_mat[3][j] = (tof_mat[1][j] + tof_mat[2][j] + tof_mat[3][j]) / 3;

                for (int8_t y = 0; y <= (8 * 7); y += 8) {
                    for (int8_t x = 7; x >= 0; x--) {
                        printf("%d %d %d\n", y/8, x, tof_mat[3][x+y]);
                    }
                    printf("\n");
                }
                printf("\n");
            }

            for (int j = 0; j < 64; j++) tof_mat[i][j] = res.distance_mm[j];
            
            i++;
        }
        sleep_ms(10);
    }

    status = vl53l5cx_stop_ranging(&dev);
    printf("stop ranging\n");
#endif

    //while (1) 
}