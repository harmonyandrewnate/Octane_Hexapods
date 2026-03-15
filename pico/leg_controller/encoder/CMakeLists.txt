add_library(encoder
    encoder.c
)

target_include_directories(encoder PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_generate_pio_header(encoder ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

target_link_libraries(encoder
    pico_stdlib
    hardware_pio
)

