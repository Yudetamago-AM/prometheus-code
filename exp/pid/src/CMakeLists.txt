add_library(prometheus-pid INTERFACE)

target_sources(prometheus-pid INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pid.c
)

target_link_libraries(prometheus-pid
    INTERFACE
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_pwm
    motor_encoder
)