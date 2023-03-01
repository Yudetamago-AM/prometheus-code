add_executable(examples
    ex_.c
        )

# pull in common dependencies
target_link_libraries(examples pico_stdlib nmea_pico hardware_uart)

# create map/bin/hex file etc.
pico_add_extra_outputs(examples)

# add url via pico_set_program_url
##example_auto_set_url(examples)

pico_enable_stdio_usb(examples 1)
pico_enable_stdio_uart(examples 0)