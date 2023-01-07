/**
 * Copyright (c) 2021 pmarques-dev @ github
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "quadrature_encoder.pio.h"

//
// ---- quadrature encoder interface example
//
// the PIO program reads phase A/B of a quadrature encoder and increments or
// decrements an internal counter to keep the current absolute step count
// updated. At any point, the main code can query the current count by using
// the quadrature_encoder_*_count functions. The counter is kept in a full
// 32 bit register that just wraps around. Two's complement arithmetic means
// that it can be interpreted as a 32-bit signed or unsigned value, and it will
// work anyway.
//
// As an example, a two wheel robot being controlled at 100Hz, can use two
// state machines to read the two encoders and in the main control loop it can
// simply ask for the current encoder counts to get the absolute step count. It
// can also subtract the values from the last sample to check how many steps
// each wheel as done since the last sample period.
//
// One advantage of this approach is that it requires zero CPU time to keep the
// encoder count updated and because of that it supports very high step rates.
//

void motor_init(uint8_t pin, uint8_t slice) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    gpio_set_function((pin + 1), GPIO_FUNC_PWM);

    pwm_set_clkdiv(slice, 1.0614809); // clkdiv = sysclock / ((wrap + 1) * f)
    pwm_set_wrap(slice, 1023);        // pwm resolution = 1024, 115kHz
    pwm_set_chan_level(slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice, PWM_CHAN_B, 0);
}

void motor_rotate(uint8_t slice, int16_t pwm) {
    // abs(pwm) < 500 (at 7.4V) means no rotation because of high(1:298) gear rates
    if (pwm > 0) {
        // forward
        pwm_set_chan_level(slice, PWM_CHAN_A, pwm);
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);
    } else {
        // backward
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, -pwm);
    }
    
    pwm_set_enabled(slice, true);
}

int main() {
    uint8_t slice_a = pwm_gpio_to_slice_num(20);
    uint8_t slice_b = pwm_gpio_to_slice_num(18);
    motor_init(20, slice_a);
    motor_init(18, slice_b);

    int new_value[2], delta[2], old_value[2] = {0};

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    const uint PIN_AB_a = 22;
    const uint PIN_AB_b = 24;

    stdio_init_all();

    PIO pio = pio0;
    const uint sm[2] = {0, 1};

    uint offset = pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm[0], offset, PIN_AB_a, 0);
    quadrature_encoder_program_init(pio, sm[1], offset, PIN_AB_b, 0);

    int16_t pwm = -1023;
    while (1) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        for (uint8_t i = 0; i < 2; i++) {
            new_value[i] = quadrature_encoder_get_count(pio, sm[i]);
            delta[i] = new_value[i] - old_value[i];
            old_value[i] = new_value[i];
        }
        
        printf("M1: pos %8d, d %6d, pwm %6d  M2: pos %8d, d %6d, pwm %6d\n", new_value[0], delta[0], pwm, new_value[1], delta[1], -pwm);
        //printf("position %8d, delta %6d, pwm %6d\n", new_value, delta, pwm);

        if (pwm >= 1023) pwm = -1023;
        motor_rotate(slice_a, pwm);
        motor_rotate(slice_b, -pwm);
        pwm++;
        sleep_ms(100);
    }
}

