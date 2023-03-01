#pragma once
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/types.h"

typedef struct GPS{
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
    int16_t rmc_checksum;
    int16_t gga_checksum;
} GPS_data_t;

#define SIZE_OF_SENTENCE 128

//
static char sentence[SIZE_OF_SENTENCE];
static bool ready = false;
static bool rmc = false;
static bool gga = false;

///functions
void get_gps (uint8_t ch, GPS_data_t * GPS_data);
//judge which data
int edit_sentence (char sentence[], GPS_data_t * GPS_data);
int get_checksum (char sentence[], int start, int finish);
//data into struct
static int into_struct_rmc (char sentence[], int i, int comma, GPS_data_t *GPS_data);
static int into_struct_gga (char sentence[], int i, int comma, GPS_data_t *GPS_data);
