add_executable(mag_cal mag_cal.c)

target_link_libraries(mag_cal pico_stdlib pico-icm20948)

pico_enable_stdio_usb(mag_cal 1)
pico_enable_stdio_uart(mag_cal 0)

pico_add_extra_outputs(mag_cal)