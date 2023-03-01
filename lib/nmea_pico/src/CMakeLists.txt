add_library(nmea_pico INTERFACE)

target_sources(nmea_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/nmea_pico.c
)

target_link_libraries(nmea_pico
    INTERFACE
    pico_stdlib
)

pico_enable_stdio_usb(nmea_pico 1)
pico_enable_stdio_uart(nmea_pico 0)