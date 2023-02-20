#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

int main(void) {
    stdio_init_all();
    printf("hello, this is pico!\n");

    gpio_init(18);
    gpio_init(19);
    gpio_init(20);
    gpio_init(21);

    gpio_set_dir(18, GPIO_OUT);
    gpio_set_dir(19, GPIO_OUT);
    gpio_set_dir(20, GPIO_OUT);
    gpio_set_dir(21, GPIO_OUT);

    gpio_put(18, 1);
    gpio_put(19, 1);
    gpio_put(20, 1);
    gpio_put(21, 1);

    while (1) tight_loop_contents();
}

