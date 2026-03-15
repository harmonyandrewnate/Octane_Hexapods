add_library(PID
    PID.c
)

target_include_directories(PID PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)


target_link_libraries(PID
    pico_stdlib
)

