cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(i2c_transmitter_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()


add_executable(i2c_transmitter
    i2c_transmitter.c
)

target_compile_definitions(i2c_transmitter PRIVATE
    SYS_CLK_MHZ=200
)

pico_enable_stdio_usb(i2c_transmitter 1)
pico_enable_stdio_uart(i2c_transmitter 1)

pico_add_extra_outputs(i2c_transmitter)

target_link_libraries(i2c_transmitter
    pico_stdlib
    hardware_i2c
)
