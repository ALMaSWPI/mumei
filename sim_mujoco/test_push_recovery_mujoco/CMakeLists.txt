add_executable(test_mujoco launch_mujoco.cc)

target_link_libraries(test_mujoco
        PUBLIC
        mujoco
        glfw
        huron::huron
)

