#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

const uint fpin = 13;
const uint led = 2;

int main(void) {
    stdio_init_all();
    printf("hello, this is \"flight_pin\"\n");

    gpio_init(fpin);
    gpio_set_dir(fpin, GPIO_IN);

    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);

    while (1) {
        if (gpio_get(fpin)) {
            // fpin == non-zero == HIGH
            printf("flight pin removed.\n");
            gpio_put(led, 0);
        } else {
            // fpin == zero == LOW
            printf("flight pin attached.\n");
            gpio_put(led, 1);
        }
        sleep_ms(10);
    }

    return 0;
}