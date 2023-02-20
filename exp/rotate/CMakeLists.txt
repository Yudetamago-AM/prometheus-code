add_executable(rotate rotate.c)

target_link_libraries(rotate pico_stdlib hardware_gpio motor_encoder)

pico_enable_stdio_usb(rotate 1)
pico_enable_stdio_uart(rotate 0)

pico_add_extra_outputs(rotate)