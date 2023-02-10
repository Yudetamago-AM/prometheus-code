#include <stdio.h>
#include "pico/stdlib.h"
#include "../src/pid.h"

static float target_rotate = 35; // 0 to 70(approx)
static uint32_t delta[2];
static uint8_t slice_left;
static uint8_t slice_right;
static PIO pio = pio0;

uint16_t limit_pwm(uint16_t pwm) {
    if (pwm < -1023) pwm = -1023;
    if (pwm > 1023) pwm = 1023;
    return pwm;
}

bool update_pid(repeating_timer_t *rt) {
    static float error_left[2];
    static float error_right[2];
    static double integral_left = 0, integral_right = 0;
    quadrature_encoder_update_delta(pio, 0, delta);
    int16_t pwml = (int16_t)pid_calc((float)(-delta[0]), target_rotate, error_left, &integral_left);
    int16_t pwmr = (int16_t)pid_calc((float)delta[1], target_rotate, error_right, &integral_right);
    pwml = limit_pwm(pwml); pwmr = limit_pwm(pwmr);
    motor_rotate(slice_left, pwml);
    motor_rotate(slice_right, pwmr);
    printf("L pwm %4d, d %3d, i %2.2f R pwm %4d, d %3d, i %2.2f\n", pwml, -delta[0], integral_left, pwmr, delta[1], integral_right);

    return true;
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("hello, this is pico!\n");

    slice_left = motor_init(20);
    slice_right = motor_init(18);

    const uint PIN_AB[2] = {22, 24}; // 22: left, 24: right

    quadrature_encoder_two_pio_init(pio, 0, PIN_AB[0], PIN_AB[1]);

    pid_init(0.01, 1.0, 0.0, 0.0);

    static repeating_timer_t timer;
    add_repeating_timer_ms(-10, &update_pid, NULL, &timer);

    while(1) {

        tight_loop_contents();
    }

    return 0;
}