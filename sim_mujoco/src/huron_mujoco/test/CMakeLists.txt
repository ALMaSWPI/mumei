project(test_huron_mujoco)

add_executable(test_mujoco launch_mujoco.cc)
add_executable(test_api api_example.cc)

target_link_libraries(test_mujoco
        PUBLIC
        huron
)

target_link_libraries(test_api
        PUBLIC
        huron
)