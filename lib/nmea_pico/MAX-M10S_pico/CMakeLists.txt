add_executable(MAX-M10S_pico
        MAX-M10S_pico.c
        )

# pull in common dependencies
target_link_libraries(MAX-M10S_pico pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(MAX-M10S_pico)

# add url via pico_set_program_url
##example_auto_set_url(MAX-M10S_pico)

pico_enable_stdio_usb(MAX-M10S_pico 1)
pico_enable_stdio_uart(MAX-M10S_pico 0)