#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/types.h"

struct GPS{
    int32_t latitude;
    int32_t longitude;
    int32_t speed;
    int32_t time;
    int16_t time_hour;
    int16_t time_min;
    int16_t time_sec;
    int32_t date;
    int16_t date_year;
    int16_t date_month;
    int16_t date_day;
    int16_t satellites;
} GPS_data;

/// \tag::uart_advanced[]

#define UART_ID uart1
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 4
#define UART_RX_PIN 5

//
#define SIZE_OF_SENTENCE 128

static int chars_rxed = 0;

//
char sentence[SIZE_OF_SENTENCE];
static bool ready = false;
static bool checksum_comparison = false;
static bool rmc = false;
static bool gga = false;
static bool rmc_ready = false;
static bool gga_ready = false;

///functions
void make_sentence (uint8_t ch);
//judge which data
int edit_sentence (char sentence[]);
//data into struct
int into_struct_rmc (char sentence[], int i, int comma);
int into_struct_gga (char sentence[], int i, int comma);
