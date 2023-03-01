add_executable(teseo_pico
    teseo_pico.c
        )

# pull in common dependencies
target_link_libraries(teseo_pico pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(teseo_pico)

# add url via pico_set_program_url
##example_auto_set_url(teseo_pico)

pico_enable_stdio_usb(teseo_pico 1)
pico_enable_stdio_uart(teseo_pico 0)