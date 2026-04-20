add_library(trajectories
    forward.c
    backward.c
    left.c
    right.c
)

target_include_directories(trajectories PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)


target_link_libraries(trajectories
)

