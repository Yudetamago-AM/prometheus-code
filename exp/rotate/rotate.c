#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "../../lib/motor_encoder/src/motor.h"

int main(void) {
    stdio_init_all();
    printf("hello, this is pico!\n");

    const uint nichrome_pin = 17;
    gpio_init(nichrome_pin);
    gpio_set_dir(nichrome_pin, GPIO_OUT);

    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);
    
    for (uint i = 0; i < 100; i++) {
        gpio_put(2, 1);
        sleep_ms(100);
        gpio_put(2, 0);
        sleep_ms(150);
    }

    gpio_put(nichrome_pin, 1);
    sleep_ms(1000);
    gpio_put(nichrome_pin, 0);
    sleep_ms(2000);

    uint slice_left = motor_init(20);
    uint slice_right = motor_init(18);

    motor_rotate(slice_left, 1023);
    motor_rotate(slice_right, 1023);

    while (1) tight_loop_contents();

    return 0;
}