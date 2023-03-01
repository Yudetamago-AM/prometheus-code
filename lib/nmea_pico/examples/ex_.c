#include "../src/nmea_pico.h"

#define GPS_UART_ID uart1

const uint gps_tx_pin = 4;
const uint gps_rx_pin = 5;

GPS_data_t GPS_data;

int main (void){
    stdio_init_all();

    uart_init(GPS_UART_ID, 9600);

    gpio_set_function(gps_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(gps_rx_pin, GPIO_FUNC_UART);

    uart_set_baudrate(GPS_UART_ID, 9600);
    uart_set_hw_flow(GPS_UART_ID, false, false);
    uart_set_format(GPS_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART_ID, true);

    gpio_init(7);
    gpio_set_dir(7, GPIO_OUT);
    gpio_put(7, 0);

    uint8_t buf[100] = {'\0'};
    uint8_t t = 0;
    while(1){

        uint16_t n;
        if (n = uart_is_readable(GPS_UART_ID)) {
            n = (n > 100) ? 100 : n;
            uart_read_blocking(GPS_UART_ID, buf,  n);
            for (uint i = 0; buf[i] != '\0'; i++) get_gps(buf[i], &GPS_data);
        }
        for (uint i = 0; i < 100; i++) buf[i] = '\0';
        
        if (t > 100) {
            t = 0;
            printf("('' -> data begins)\n");
            printf("date:%d/%d/%d\n", GPS_data.date_year, GPS_data.date_month, GPS_data.date_day);
            printf("time:%d:%d:%d\n", GPS_data.time_hour, GPS_data.time_min, GPS_data.time_sec);
            printf("latitude:%d\n", GPS_data.latitude);
            printf("longitude:%d\n", GPS_data.longitude);
            printf("speed(knot):%d\n", GPS_data.speed);
            printf("satellites:%d\n", GPS_data.satellites);
            printf("('' ->   data_ends)\n");
            printf("\n");
        }
        
        t++;
        sleep_ms(10);
    }

    return 0;
}