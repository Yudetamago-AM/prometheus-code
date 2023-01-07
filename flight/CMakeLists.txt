add_executable(flight flight.c)

target_link_libraries(flight pico_stdlib hardware_i2c vl53l5cx pico-icm20948 MadgwickAHRS)

pico_enable_stdio_usb(flight 1)
pico_enable_stdio_uart(flight 0)

pico_add_extra_outputs(flight)