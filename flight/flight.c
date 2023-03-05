#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "../lib/prometheus-pin/prometheus-pin.h"
#include "../lib/motor_encoder/src/motor.h"
#include "../lib/icm20948/src/pico-icm20948.h"
#include "../lib/icm20948/MadgwickAHRS/MadgwickAHRS.h"
#include "../lib/vl53l5cx/api/vl53l5cx_api.h"
#include "../lib/health_monitor/src/health.h"
#include "../lib/pico-eeprom-i2c/src/eeprom.h"
#include "../lib/gnss/nmeap-0.3/inc/nmeap.h"
#include "../exp/pid/src/pid.h"
#include "util.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(a) (((a)<0)?(-(a)):(a))

#define MODE_WAIT    1
#define MODE_LANDING 2
#define MODE_GNSS    3
#define MODE_TOF     4
#define MODE_GOAL    5
#define MODE_FORWARD_LANDING 6
#define MODE_FORWARD_TOF     7

// 9-axis
icm20948_config_t config = {0x68, 0x0C, i2c1};
madgwick_ahrs_t filter = {0.5f, {1.0f, 0.0f, 0.0f, 0.0f}};
icm20984_data_t icmdata;
static float euler[3] = {0};

// 投下前手順
/*
0. バッテリ電圧を確認する
1. selftestのクリアを確認する
2. mode_nowをMODE_WAITにする
3. 白黒か黄色でゴール座標
4. 白黒か黄色で地磁気
*/

// SETTINGS
// 白黒 COM5
const int32_t goal_longitude = 130960119;
const int32_t goal_latitude  =  30374279;

// 黄色 COM3
//const int32_t goal_longitude = 130960121;
//const int32_t goal_latitude  =  30374280;

// 初期モード
static uint mode_now = MODE_WAIT;

const int32_t wait_count = 3 * 100; // 30秒
const float angle_th = 0.39; //前45度，GPS誘導
const uint16_t long_per_res = 111; // 種子島
const uint16_t lat_per_les = 96;
//

static repeating_timer_t timer;
static uint32_t logh = 0;

// tof
i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};
VL53L5CX_Configuration tof_dev;
VL53L5CX_ResultsData tof_res;
static uint8_t tof_isready;

// gps
nmeap_context_t nmea;
nmeap_gga_t gga;
static bool gps_isready = false;

// motor
static uint8_t slice_left;
static uint8_t slice_right;
PIO pio = pio0;
uint32_t delta[2];

static uint8_t logbuf[3000] = {0};

bool update_all(repeating_timer_t *rt);
bool is_goal();
void calc_gps(float *direction, int32_t *distance, int32_t long_now, int32_t lat_now);
//void calc_gps(float *dir, float *dist, double lon, double lat);

#if 1
//int16_t elog_ret;
void elog(uint8_t *src, int16_t len) {
    if (logh+8+len > 3000) return;
    uint8_t t[100] = {'\0'};
    snprintf(t, 8+len, "%06d,%s", gga.time, src);
    //printf("%s", t);
    for (int16_t i = 0; i < (len+8); i++) {
        logbuf[i+logh] = t[i];
    }
    //elog_ret = eeprom_write_multi(i2c1, 0x50, logh, t, len+7);
    logh += 7;
    logh += len;
}
#endif

void on_uart_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        nmeap_parse(&nmea, ch);
    }
}

// icm-20948
void icm20948_calc_data(float accel_g[3], float gyro_dps[3]) {
    accel_g[0] = -(float)icmdata.accel_raw[1] / 16384.0f; // センサー座標でx
    accel_g[1] = (float)icmdata.accel_raw[0] / 16384.0f; // センサー座標でy
    accel_g[2] = (float)icmdata.accel_raw[2] / 16384.0f; // センサー座標でz
    gyro_dps[0] = -(float)icmdata.gyro_raw[1] / 131.0f; // roll
    gyro_dps[1] = (float)icmdata.gyro_raw[0] / 131.0f; // pitch
    gyro_dps[2] = (float)icmdata.gyro_raw[2] / 131.0f; // yaw
}

float icm20948_calc_yawmag() {
    // 地磁気をセンサー座標系に合わせる
    float yawmag;
    int16_t tmp = icmdata.mag_raw[0];
    icmdata.mag_raw[0] = icmdata.mag_raw[1]; // x
    icmdata.mag_raw[1] = tmp;             // y
    icmdata.mag_raw[2] = -icmdata.mag_raw[2];// z
    float mx = ((float)icmdata.mag_raw[0])*cos(euler[1]) + ((float)icmdata.mag_raw[1])*sin(euler[0])*sin(euler[1]) + ((float)icmdata.mag_raw[2])*cos(euler[0])*sin(euler[1]);
    float my = ((float)icmdata.mag_raw[1])*cos(euler[0]) - ((float)icmdata.mag_raw[2])*sin(euler[0]);
    yawmag = atan2f(my, mx);
}

void icm20948_read_all(/* icm20948_config_t *config, icm20984_data_t *icmdata */) {
    icm20948_read_cal_accel(&config, &icmdata.accel_raw[0], &icmdata.accel_bias[0]);
    icm20948_read_cal_gyro(&config, &icmdata.gyro_raw[0], &icmdata.gyro_bias[0]);
    icm20948_read_cal_mag(&config, &icmdata.mag_raw[0], &icmdata.mag_bias[0]);
}

void q2e(madgwick_ahrs_t *icmdata, float euler[]) {
    // 0: roll, 1: pitch, 2: yaw
    euler[0] = -1.0f * asinf(2.0f * (icmdata->q[1]) * (icmdata->q[3]) + 2.0f * (icmdata->q[0]) * (icmdata->q[2]));
    euler[1] = atan2f(2.0f * (icmdata->q[2]) * (icmdata->q[3]) - 2.0f * (icmdata->q[0]) * (icmdata->q[1]), 2.0f * (icmdata->q[0]) * (icmdata->q[0]) + 2.0f * (icmdata->q[3]) * (icmdata->q[3]) - 1.0f);
    euler[2] = atan2f(2.0f * (icmdata->q[1]) * (icmdata->q[2]) - 2.0f * (icmdata->q[0]) * (icmdata->q[3]), 2.0f * (icmdata->q[0]) * (icmdata->q[0]) + 2.0f * (icmdata->q[1]) * (icmdata->q[1]) - 1.0f);
    return;
}

// motor
int16_t limit_pwm_1023(int16_t pwm) {
    pwm = constrain(pwm, -1023, 1023);
    return pwm;
}

int16_t limit_pwm_800(int16_t pwm) {
    pwm = constrain(pwm, -800, 800);
    return pwm;
}

// gps
static void gpgga_callout(nmeap_context_t *context, void *data, void *user_data) {
    gps_isready = true;
}

#if 1
void calc_gps(float *direction, int32_t *distance, int32_t long_now, int32_t lat_now) {
    int32_t dx = ((goal_longitude - long_now) * long_per_res);//0.000001度で0.092m(京田辺)，0.085m(能代)より，単位メートル
    int32_t dy = ((goal_latitude - lat_now) * lat_per_les);//0.000001度で0.111m(111)より0.1
    
    if (dx == 0 && dy == 0) *direction = 0;
    else *direction = atan2(dx, dy);//意図的にdx, dyの順，というのも，北基準だから．
    *distance = approx_distance(dx, dy) / 10;//単位:cm
    *distance = abs(*distance);
    printf("calc_gps: dx %d, dy %d, dir %f, dist %d\n", dx, dy, *direction, *distance);
    printf("long_now %d, lat_now %d, long_goal %d, lat_goal %d\n", long_now, lat_now, goal_longitude, goal_latitude);
}
#endif

#if 0
void calc_gps(float *dir, float *dist, double lon, double lat) {
    double A  = 6378137;
    double dlong = (goal_longitude - lon);
    double dlat  = (goal_latitude - lat);
    double dx = (dlong * cos(lat) * A);
    double dy = (dlat * A);
    *dir = atan2f(dy, dx);
    *dist = sqrt(dx*dx + dy*dy);
    printf("calc_gps: dx %f, dy %f, dir %f, dist %f\n", dx, dy, *dir, *dist);
    printf("long_now %f, lat_now %f, long_goal %f, lat_goal %f\n", lon, lat, goal_longitude, goal_latitude);
}
#endif

void mode_wait() {
    static bool is_removed = false;
    static int32_t cnt = 0;
    static bool isfirst = true;
    static uint8_t str[] = "fpin removed\n";
    static uint8_t str2[] = "wait end\n";

    if (cnt > wait_count) {
        mode_now = MODE_LANDING;
        printf("cnt > wait_count, cnt: %d\n", cnt);
        elog(str2, 9);
        return;
    }
    
    if (gpio_get(flight_pin)) {
#if 1
        if (isfirst) {
            isfirst = false;
            elog(str, 13);
        }
#endif
        // removed
        cnt++;
        printf("cnt: %d\n", cnt);
    }
}

void mode_landing() {
    static bool isfirst = true;
    static uint8_t str[]  = "nichr heat st\n";
    static uint8_t str2[] = "nichr heat end\n";
    if (isfirst) {
        isfirst = false;
        elog(str, 14);
    }
    printf("mode_landing\n");
    static int32_t count = 0;
    gpio_put(nichrome_pin, 1);

    count++;
    if (count > 150) {
        gpio_put(nichrome_pin, 0);
        printf("nichrome end\n");
        elog(str2, 15);
        mode_now = MODE_FORWARD_LANDING;
    }

}

void mode_forward_landing() {
    static uint8_t str[] = "mf 2s\n";
    static int32_t cnt = 0;

    motor_rotate(slice_left, 800);
    motor_rotate(slice_right, 800);

    if (cnt > 200) {
        elog(str, 6);
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, 0);
        mode_now = MODE_GNSS;
    }
    cnt++;
}

void mode_gnss() {
    static uint8_t str[] = "gnss\n";
    static uint8_t str2[20];
    static uint8_t str3[11];
    static bool isfirst = true;
    printf("mode_gnss\n");
    static float dir, yaw;
    static int32_t dist;
    static int16_t times = 0;
    static int16_t ttimes = 0;
    static float error[2] = {0}, integral = 0;

    if (isfirst) {
        isfirst = false;
        elog(str, 5);
    }

    if (gps_isready) {
        gps_isready = false;
        printf("gps_isready: true, %.6f,%.6f, sat %d\n", gga.latitude, gga.longitude, gga.satellites);
        int32_t lon = (int32_t)(gga.longitude * 1000000);
        int32_t lat = (int32_t)(gga.latitude * 1000000);
        calc_gps(&dir, &dist, lon, lat);
        if (dist < 70) {
            mode_now = MODE_FORWARD_TOF;
        }
        snprintf(str2, 19, "%d,%d\n", lat, lon);
        elog(str2, 18);
        //calc_gps(&dir, &dist, gga.longitude, gga.latitude);
        //if (gga.longitude == 0.0) is_valid = false;
    }
    if ((times == 10)) {
        times = 0;
        yaw = icm20948_calc_yawmag();
        int16_t l, r;
        //printf("yaw %f dir %f dir-th %f dir+t %f\n", yaw, dir, dir-angle_th, dir+angle_th);
        if ((yaw > dir-angle_th) && (yaw < dir+angle_th)) {
            l = 900; r = 900;
            printf("straight\n");
            motor_rotate(slice_left, 900);
            motor_rotate(slice_right, 900);
        } else if (yaw < (dir-angle_th)) {
            l = 900; r = 650;
            // 右旋回
            printf("right\n");
            motor_rotate(slice_left, 900);
            motor_rotate(slice_right, 650);
        } else if (yaw > (dir+angle_th)) {
            // 左旋回
            //pwm = -pwm;
            printf("left\n");
            l = 650; r = 900;
            motor_rotate(slice_left, 650);
            motor_rotate(slice_right, 900);
        } else {
            printf("sikatanaku right\n");
            motor_rotate(slice_left, 900);
            motor_rotate(slice_right, 650);
        }
#if 1
        if (ttimes == 10) {
            ttimes = 0;
            snprintf(str3, 11, "mr,%d%d\n", l, r);
            elog(str3, 10);
        }
        ttimes++;
#endif
    }
    times++;
}

void mode_forward_tof() {
    static int32_t cnt = 0;

    motor_rotate(slice_left, 800);
    motor_rotate(slice_right, 800);

    if (cnt > 200) {
        motor_rotate(slice_left, 0);
        motor_rotate(slice_right, 0);
        mode_now = MODE_TOF;
    }
    cnt++;
}

void mode_tof() {
    static int8_t ct = 0;
    static int16_t tof_mat[4][64];
    static int32_t history[20] = {4000};
    int8_t lflg = 0;
    uint8_t status;
    static bool isfirst = true;
    static int8_t lflg_cnt = 0;
    printf("mode_tof\n");

#if 1
    status = vl53l5cx_check_data_ready(&tof_dev, &tof_isready);
    printf("st: %d\n", status);
    if (tof_isready) {
        //printf("tof ready\n");
        vl53l5cx_get_ranging_data(&tof_dev, &tof_res);

        if (ct == 2) {

            ct = 0;
            for (int j = 0; j < 64; j++) tof_mat[3][j] = (tof_mat[1][j] + tof_mat[2][j] + tof_mat[3][j]) / 3;
            for (int8_t j = 0; j < 64; j++) {
                if (tof_mat[3][j] < 600) lflg++; 
            }
            printf("tof32: %d, lflg: %d\n", tof_mat[3][32], lflg);

#if 0
            int32_t avg;
            for (int8_t j = 0; j < 64; j++) avg += tof_mat[3][j];
            avg /= 64;
            printf("avg: %d\n", avg);

#endif
            if (tof_mat[3][32] < 200) {
                bool isgoal = is_goal();
                if (isgoal) {
                    mode_now = MODE_GOAL;
                    return;
                } else {
                    mode_now = MODE_GNSS;
                    return;
                }
            }
            if (lflg > 32) {
                printf("tof straight\n");
                motor_rotate(slice_left, 950);
                motor_rotate(slice_right, 950);
            } else {
                printf("tof_left-only\n");
                motor_rotate(slice_left, 950);
                motor_rotate(slice_right, 0);
            }

        }
        for (int j = 0; j < 64; j++) tof_mat[ct][j] = tof_res.distance_mm[j];
        ct++;

    }
#endif

#if 0
/*
    if (isfirst) {
    printf("cancel rt\n");
    isfirst = false;
    cancel_repeating_timer(&timer);
    }
*/

    status = vl53l5cx_check_data_ready(&tof_dev, &tof_isready);
    printf("st: %d\n", status);
    if (tof_isready) {
        //printf("tof ready\n");
        vl53l5cx_get_ranging_data(&tof_dev, &tof_res);

        // qsortからの中央値だして比較？
        if (ct == 2) {
            lflg = 0;
            ct = 0;
            for (int j = 0; j < 64; j++) tof_mat[3][j] = (tof_mat[1][j] + tof_mat[2][j] + tof_mat[3][j]) / 3;

            for (int j = 0; j < 20-1; j++) history[j] = history[j+1];
            for (int j = 0; j < 64; j++) history[19] += tof_mat[3][j];
            history[19] /= 64;

            printf("h[19] %d\n", history[19]);

            if (history[19] < 1000) {
                motor_rotate(slice_left, 950);
                motor_rotate(slice_right, 950);
                return;
            } 

            if (history[19] < 200) {
                bool isgoal = is_goal();
                if (is_goal) {./
                    mode_now = MODE_GOAL;
                    return;
                } else {
                    mode_now = MODE_GNSS;
                    return;
                }
            }

            for (int8_t j = 0; j < 19; j++) {
                if (history[j] > history[19]) lflg++;
            }
            //printf("h1: %d h2: %d h3: %d h18: %d\n", history[1], history[2], history[3], history[18]);
            printf("lflg: %d, h19: %d\n", lflg, history[19]);

            if (lflg > 18) {
                motor_rotate(slice_left, 800);
                motor_rotate(slice_right, 800);
                printf("staright!\n");
            } else {
                motor_rotate(slice_left, 800);
                motor_rotate(slice_right, 0);
                printf("rotate\n");
            }
        }

        for (int j = 0; j < 64; j++) tof_mat[ct][j] = tof_res.distance_mm[j];
        ct++;
        }
    
#endif
}

bool is_goal() {
    printf("is_goal\n");
    static uint8_t str[18];
    float dir;
    int32_t dist;
    bool gps = false, tof = false;
    int8_t tof_count = 0;
    int16_t tof_mat[64];
    uint8_t tof_isready;

        if (gps_isready) {
            int32_t lon = (int32_t)(gga.longitude * 1000000);
            int32_t lat = (int32_t)(gga.latitude * 1000000);
            calc_gps(&dir, &dist, lon, lat);
            if (dist < 400) gps = true;
        }

        //vl53l5cx_check_data_ready(&tof_dev, &tof_isready);
        //if (tof_isready) {
            //vl53l5cx_get_ranging_data(&tof_dev, &tof_res);
            for (int8_t i = 0; i < 64; i++) {
                if (tof_res.distance_mm[i] < 150) tof_count++;
            }
            if (tof_count > 35) tof = true;
        //}

        snprintf(str, 17, "isg,d%04d,t%02d\n", dist, tof_count);
        elog(str, 16);

        if (gps && tof) return true;
        else if (gps_isready && tof_isready) return false;


}

void mode_goal() {
    cancel_repeating_timer(&timer);
    static bool isfirst = true;
    static uint8_t str[] = "goal!\n";
    if (isfirst) {
        isfirst = false;
        elog(str, 6);
    }

    eeprom_write_multi(i2c1, 0x50, 0x0000, logbuf, logh);
    printf("mode_goal\n");
    while(1) tight_loop_contents();
}

// overall
bool update_all(repeating_timer_t *rt) {
    static float accel_g[3] = {0}, gyro_dps[3] = {0};
    uint8_t buf[11];
    icm20948_read_all();
    icm20948_calc_data(accel_g, gyro_dps);
    MadgwickAHRSupdateIMU(&filter, gyro_dps[0], gyro_dps[1], gyro_dps[2], accel_g[0] * 9.8, accel_g[1] * 9.8, accel_g[2] * 9.8);
    q2e(&filter, euler);

    switch (mode_now)
    {
    case MODE_WAIT:
        mode_wait();
        break;
    case MODE_LANDING:
        mode_landing();
        break;
    case MODE_FORWARD_LANDING:
        mode_forward_landing();
        break;
    case MODE_GNSS:
        mode_gnss();
        break;
    case MODE_FORWARD_TOF:
        mode_forward_tof();
        break;
    case MODE_TOF:
        mode_tof();
        break;
    case MODE_GOAL:
        mode_goal();
        break;
    }
}

int main(void) {
    stdio_init_all();

    // gnss on
    gpio_init(gnss_vcc_pin);
    gpio_set_dir(gnss_vcc_pin, GPIO_OUT);
    gpio_put(gnss_vcc_pin, 0);

    // flight pin
    gpio_init(flight_pin);
    gpio_set_dir(flight_pin, GPIO_IN);

    // i2c0
    // vl53l5cx
    gpio_init(tof_vcc_pin);
    gpio_set_dir(tof_vcc_pin, GPIO_OUT);
    gpio_put(tof_vcc_pin, 0);
    sleep_ms(100);

    i2c_init(&vl53l5cx_i2c, 400 * 1000);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);

    uint8_t tof_status, is_ready, is_alive;

    tof_dev.platform.address = 0x29;
    tof_dev.platform.i2c     = &vl53l5cx_i2c;
    tof_status = vl53l5cx_is_alive(&tof_dev, &is_alive);
    tof_status = vl53l5cx_init(&tof_dev);
    if (tof_status) printf("vl53l5cx: ULD load failed\n");
    if (!tof_status) printf("vl53l5cx: OK\n");
    else printf("vl53l5cx: NG\n");
    // 8x8
    tof_status = vl53l5cx_set_resolution(&tof_dev, VL53L5CX_RESOLUTION_8X8);
    if (tof_status) printf("err: set_resolution\n");
    // 5Hz
    tof_status = vl53l5cx_set_ranging_frequency_hz(&tof_dev, 15);
    if (tof_status) printf("err: set_ranging_frequency_hz\n");
    // strongest -> closest
    tof_status = vl53l5cx_set_target_order(&tof_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    if (tof_status) printf("err: set_target_order\n");
    // continuous
    tof_status = vl53l5cx_set_ranging_mode(&tof_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
    if (tof_status) printf("err: set_ranging_mode\n");
    tof_status = vl53l5cx_start_ranging(&tof_dev);
    if (tof_status) printf("err: start_ranging\n");

    // i2c1
    // icm-20948
    icm20948_config_t config = {0x68, 0x0C, i2c1};
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    if (icm20948_init(&config) == 0) printf("icm20948: OK\n");
    else printf("icm20948: NG\n");
    icm20948_cal_gyro(&config, &icmdata.gyro_bias[0]);
    printf("calibrated gyro: %d %d %d\n", icmdata.gyro_bias[0], icmdata.gyro_bias[1], icmdata.gyro_bias[2]);
    icm20948_cal_accel(&config, &icmdata.accel_bias[0]);
    printf("calibrated accel: %d %d %d\n", icmdata.accel_bias[0], icmdata.accel_bias[1], icmdata.accel_bias[2]);


// タイヤ白黒機体 
icmdata.mag_bias[0] = 110; icmdata.mag_bias[1] = 209; icmdata.mag_bias[2] = -308;
// タイヤ黄色機体
//icmdata.mag_bias[0] = 41; icmdata.mag_bias[1] = 847; icmdata.mag_bias[2] = -610;


    // health check
    float current, voltage;
    health_init(current_sense_pin, voltage_sense_pin);
    current = health_current_read(current_sense_pin);
    voltage = health_voltage_read(voltage_sense_pin);
    printf("health: %f V, %f A\n", voltage, current);

    // nichrome
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    // led
    gpio_init(led_pin);

    // motor, encoder
    slice_left = motor_init(motor_left_a_pin);
    slice_right = motor_init(motor_right_a_pin);
    
    //quadrature_encoder_two_pio_init(pio, 0, encoder_left_a_pin, encoder_right_a_pin);
    motor_rotate(slice_left, 0);
    motor_rotate(slice_right, 0);

    // gnss
    uart_init(uart1, 9600);
    gpio_set_function(gnss_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(gnss_rx_pin, GPIO_FUNC_UART);
    uart_set_baudrate(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    //uart_set_fifo_enabled(uart1, true);
    uart_set_fifo_enabled(uart1, false);
    irq_set_exclusive_handler(UART1_IRQ, on_uart_rx);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);

    nmeap_init(&nmea, NULL);
    nmeap_addParser(&nmea, "GNGGA", nmeap_gpgga, gpgga_callout, &gga);

    add_repeating_timer_ms(-10, &update_all, NULL, &timer);

    while (1) {
        tight_loop_contents();
    }
}