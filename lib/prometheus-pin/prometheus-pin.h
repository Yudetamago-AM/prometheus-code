#ifndef PROMETHEUS_PIN_H
#define PROMETHEUS_PIN_H
#include <stdio.h>
#include "pico/stdlib.h"

// motor
const uint motor_right_a_pin = 18;
//const uint motor_right_b_pin = 19;
const uint motor_left_a_pin  = 20;
//const uint motor_left_b_pin  = 21;

// encoder
const uint encoder_right_a_pin = 24;
//const uint encoder_right_b = 25;
const uint encoder_left_a_pin  = 22;
//const uint encoder_left_b  = 23;

// i2c0: vl53l5cx
const uint i2c0_sda_pin       = 8; // tof
const uint i2c0_scl_pin       = 9; // tof
// vl53l5cx
const uint tof_vcc_pin        = 11; // input LOW to ON

// i2c1: icm-20948, eeprom
const uint i2c1_sda_pin       = 14; // imu, eeprom
const uint i2c1_scl_pin       = 15; // imu, eeprom

// uart: gnss
const uint gnss_tx_pin        = 4;
const uint gnss_rx_pin        = 5;
const uint gnss_vcc_pin       = 7; // intput LOW to ON

// current sense
// I = Imonitor * 0.15 (A)
const uint current_sense_pin  = 28;

// voltage sense
// V = Vmonitor * 4.0 (V)
const uint voltage_sense_pin  = 29;

// status led
const uint led_pin            = 2;

// nichrome
const uint nichrome_pin       = 17;

#endif