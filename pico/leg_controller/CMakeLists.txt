cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(leg_controller_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()


add_subdirectory(encoder)
add_subdirectory(motor_driver)
add_subdirectory(stepper_driver)
add_subdirectory(PID)


add_executable(leg_controller
    leg_controller.c
)

target_compile_definitions(leg_controller PRIVATE
    SYS_CLK_MHZ=200
)

pico_enable_stdio_usb(leg_controller 1)
pico_enable_stdio_uart(leg_controller 1)

pico_add_extra_outputs(leg_controller)

target_link_libraries(leg_controller 
    pico_stdlib
    hardware_pio
    encoder
    motor_driver
    stepper_driver
    PID
)
