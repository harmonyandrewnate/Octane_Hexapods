add_library(motor_driver
    motor_driver.c
)

target_include_directories(motor_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)


target_link_libraries(motor_driver
    pico_stdlib
)

