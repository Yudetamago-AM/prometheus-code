add_executable(flight flight.c)

target_link_libraries(flight 
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_pio
    hardware_i2c
    hardware_uart
    prometheus-pin
    motor_encoder
    health_monitor
    pico-icm20948
    MadgwickAHRS
    prometheus-pid
    eeprom
    gnss
    vl53l5cx
)


pico_enable_stdio_usb(flight 1)
pico_enable_stdio_uart(flight 0)

pico_add_extra_outputs(flight)
