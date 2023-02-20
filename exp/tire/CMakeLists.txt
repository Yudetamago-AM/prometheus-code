add_executable(tire tire.c)

target_link_libraries(tire pico_stdlib hardware_gpio)

pico_enable_stdio_usb(tire 1)
pico_enable_stdio_uart(tire 0)

pico_add_extra_outputs(tire)