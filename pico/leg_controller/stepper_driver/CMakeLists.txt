add_library(stepper_driver
    stepper_driver.c
)

target_include_directories(stepper_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_generate_pio_header(stepper_driver ${CMAKE_CURRENT_LIST_DIR}/stepper_driver.pio)

target_link_libraries(stepper_driver
    pico_stdlib
    pico_time
    hardware_gpio
    hardware_pwm
    hardware_pio
    hardware_clocks
)

