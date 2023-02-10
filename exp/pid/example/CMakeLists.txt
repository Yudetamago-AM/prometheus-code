add_executable(ex_pid ex_pid.c)

target_link_libraries(ex_pid pico_stdlib prometheus-pid)

pico_enable_stdio_usb(ex_pid 1)
pico_enable_stdio_uart(ex_pid 0)

pico_add_extra_outputs(ex_pid)