add_library(stepper_driver
    stepper_driver.c
)

target_include_directories(stepper_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)


target_link_libraries(stepper_driver
    pico_stdlib
)

