cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(test_mujoco)

find_package(mumei REQUIRED)

add_executable(test_mujoco test_mujoco.cc)

target_link_libraries(test_mujoco
        PUBLIC
        mumei::mumei
)

