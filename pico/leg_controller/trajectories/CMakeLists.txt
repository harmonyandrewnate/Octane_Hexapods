add_library(trajectories
    forward.c
)

target_include_directories(trajectories PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)


target_link_libraries(trajectories
)

