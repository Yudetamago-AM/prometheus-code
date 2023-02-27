add_library(prometheus-pin INTERFACE)

target_sources(prometheus-pin INTERFACE
${CMAKE_CURRENT_LIST_DIR}/prometheus-pin.c
)