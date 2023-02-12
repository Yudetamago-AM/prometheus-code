add_executable(ex2_enkaigei ex2_enkaigei.c)

target_link_libraries(ex2_enkaigei pico_stdlib prometheus-pid pico-icm20948 MadgwickAHRS)

pico_enable_stdio_usb(ex2_enkaigei 1)
pico_enable_stdio_uart(ex2_enkaigei 0)

pico_add_extra_outputs(ex2_enkaigei)