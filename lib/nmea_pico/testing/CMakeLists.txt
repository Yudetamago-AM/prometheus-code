add_executable(testing
    testing.c
        )

# pull in common dependencies
target_link_libraries(testing pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(testing)

# add url via pico_set_program_url
##example_auto_set_url(testing)

pico_enable_stdio_usb(testing 1)
pico_enable_stdio_uart(testing 0)