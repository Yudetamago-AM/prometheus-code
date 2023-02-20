add_executable(flight_pin flight_pin.c)

target_link_libraries(flight_pin pico_stdlib hardware_gpio)

pico_enable_stdio_usb(flight_pin 1)
pico_enable_stdio_uart(flight_pin 0)

pico_add_extra_outputs(flight_pin)