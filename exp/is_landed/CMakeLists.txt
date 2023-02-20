add_executable(is_landed is_landed.c)

target_link_libraries(is_landed pico_stdlib hardware_gpio hardware_i2c pico-icm20948)

pico_enable_stdio_usb(is_landed 1)
pico_enable_stdio_uart(is_landed 0)

pico_add_extra_outputs(is_landed)