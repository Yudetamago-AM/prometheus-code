add_executable(cone_m cone_m.c)

target_link_libraries(cone_m pico_stdlib hardware_i2c hardware_pwm hardware_pio vl53l5cx prometheus-pid)

pico_enable_stdio_usb(cone_m 1)
pico_enable_stdio_uart(cone_m 0)

pico_add_extra_outputs(cone_m)
