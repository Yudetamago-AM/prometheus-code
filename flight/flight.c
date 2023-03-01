#include <stdio.h>
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
#include "../lib/nmea_pico/src/nmea_pico.h"
#include "../lib/health_monitor/src/health.h"
#include "../lib/pico-eeprom-i2c/src/eeprom.h"

#define ANGLE_TH 0.39//前45度，GPS誘導
#define LONGITUDE_PER_RES 111
#define LATITUDE_PER_RES 92

// tof
i2c_inst_t vl53l5cx_i2c = {i2c0_hw, false};

// 9-axis
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

    // i2c0
    // vl53l5cx
    gpio_init(tof_vcc_pin);
    gpio_set_dir(tof_vcc_pin, GPIO_OUT);
    gpio_put(tof_vcc_pin, 0);
    sleep_ms(100);

    i2c_init(&vl53l5cx_i2c, 400 * 1000);
    gpio_set_function(i2c0_scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c0_sda_pin, GPIO_FUNC_I2C);

    uint8_t status, is_ready, is_alive;
    VL53L5CX_Configuration dev;
    VL53L5CX_ResultsData res;

    dev.platform.address = 0x29;
    dev.platform.i2c     = &vl53l5cx_i2c;
    status = vl53l5cx_is_alive(&dev, &is_alive);
    status = vl53l5cx_init(&dev);
    if (status) printf("vl53l5cx: ULD load failed\n");
    if (!status) printf("vl53l5cx: OK\n");
    else printf("vl53l5cx: NG\n");

    // i2c1
    // icm-20948
    icm20948_config_t config = {0x68, 0x0C, i2c1};
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(i2c1_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(i2c1_scl_pin, GPIO_FUNC_I2C);
    if (icm20948_init(&config) == 0) printf("icm20948: OK\n");
    else printf("icm20948: NG\n");

    // eeprom
    uint8_t wbuf[10] = {0};
    uint8_t rbuf[10] = {'0'};
    for (uint8_t i = 0; i < 10; i++) wbuf[i] = 33 + i;
    printf("write:\n");
    for (uint8_t i = 0; i < 10; i++) printf("%c", wbuf[i]);
    printf("\n");
    int16_t ret = eeprom_write_multi(i2c1, 0x50, 0x00000, wbuf, 10);
    if (ret > 0) printf("eeprom write: OK\n");
    else printf("eeprom write: NG\n");
    ret = eeprom_read(i2c1, 0x50, 0x0000, rbuf, 10);
    if (ret > 0) printf("eeprom read: OK\n");
    else printf("eeprom read: NG\n");
    for (int8_t i = 0; i < 10; i++) printf("%c", rbuf[i]);
    printf("\n");

    // gnss
    gpio_init(gnss_vcc_pin);
    gpio_set_dir(gnss_vcc_pin, GPIO_OUT);
    gpio_put(gnss_vcc_pin, 0);

    uart_init(uart1, 9600);
    gpio_set_function(gnss_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(gnss_rx_pin, GPIO_FUNC_UART);
    uart_set_baudrate(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    
    sleep_ms(5000);

    uint8_t buf[100] = {'\0'};
    int16_t n;
    if (n = uart_is_readable(uart1)) {
        uart_read_blocking(uart1, buf, 50);
        printf("%s\n", buf);
    }
    if (n > 0) printf("gnss: OK, n: %d\n", n);
    else printf("gnss: NG, n: %d\n", n);

    // motor, encoder
    uint8_t sleft = motor_init(motor_left_a_pin);
    uint8_t sright = motor_init(motor_right_a_pin);
    PIO pio = pio0;
    uint32_t delta[2];
    motor_rotate(sleft, 800);
    motor_rotate(sright, 800);
    sleep_ms(1000);
    quadrature_encoder_two_pio_init(pio, 0, encoder_left_a_pin, encoder_right_a_pin);
    sleep_ms(10);
    quadrature_encoder_update_delta(pio, 0, delta);
    if (delta[0] > 30) printf("motor_left: OK, delta: %d\n", delta[0]);
    else printf("motor_left: NG\n");
    if (delta[1] > 30) printf("motor_right: OK, delta: %d\n", delta[1]);
    else printf("motor_right: NG\n");
    motor_rotate(sleft, 0);
    motor_rotate(sright, 0);

    // health check
    float current, voltage;
    health_init(current_sense_pin, voltage_sense_pin);
    current = health_current_read(current_sense_pin);
    voltage = health_voltage_read(voltage_sense_pin);
    printf("health: %f V, %f A\n", voltage, current);

    // nichrome
    printf("check nichrome\n");
    sleep_ms(1000);
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);
    gpio_put(nichrome_pin, 1);
    sleep_ms(500);
    gpio_put(nichrome_pin, 0);
    sleep_ms(2000);

    // led
    gpio_init(led_pin);

    while (1) {
        // GPSのデータ取得

        // 
    }

}