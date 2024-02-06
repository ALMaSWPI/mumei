cmake_minimum_required(VERSION 3.16)
project(mujoco_huron)
add_executable(test_mujoco launch_mujoco.cc)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
message(glfw_INCLUDE_DIRS="${glfw3_INCLUDE_DIRS}")
target_link_libraries(test_mujoco
        PUBLIC
        mujoco
        glfw
)

