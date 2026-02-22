cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_esc_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(esc_controller
    src/main.cpp
)

target_link_libraries(esc_controller
    pico_stdlib
    hardware_pwm
    hardware_adc
)

pico_add_extra_outputs(esc_controller)
