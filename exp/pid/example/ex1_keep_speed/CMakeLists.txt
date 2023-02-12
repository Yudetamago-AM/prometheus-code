add_executable(ex1_pid ex1_pid.c)

target_link_libraries(ex1_pid pico_stdlib prometheus-pid)

pico_enable_stdio_usb(ex1_pid 1)
pico_enable_stdio_uart(ex1_pid 0)

pico_add_extra_outputs(ex1_pid)