#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../../lib/vl53l5cx/api/vl53l5cx_api.h"

const uint tof_sda = 8;
const uint tof_scl = 9;
const uint tof_vcc = 11; // LOW to power on
i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};

int32_t tof_mat[4][64] = {0};

int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    printf("hello, this is \"cone\"\n");

    gpio_init(tof_vcc);
    gpio_set_dir(tof_vcc, GPIO_OUT);
    gpio_put(tof_vcc, 0);

    i2c_init(&vl53l5cx_i2c, 400 * 1000);
    gpio_set_function(tof_scl, GPIO_FUNC_I2C);
    gpio_set_function(tof_sda, GPIO_FUNC_I2C);

    //gpio_pull_up(tof_scl);
    //gpio_pull_up(tof_sda);

    uint8_t status, is_ready, is_alive;
    VL53L5CX_Configuration dev;
    VL53L5CX_ResultsData res;

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
#if 0
    // continuous -> autonomous
    status = vl53l5cx_set_ranging_mode(&dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);
    if (status) printf("err: set_ranging_mode\n");
    // integration time: 2 - 1000 ms
    status = vl53l5cx_set_integration_time_ms(&dev, 100);
    if (status) printf("err: set_integration_time\n");
#endif
    status = vl53l5cx_start_ranging(&dev);
    if (status) printf("err: start_ranging\n");

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

    //while (1) tight_loop_contents();
    return 0;
}
