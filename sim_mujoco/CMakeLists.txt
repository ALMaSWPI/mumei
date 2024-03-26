cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

project(sim_mujoco)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/..)
find_package(huron REQUIRED)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(
        ${huron_INCLUDE_DIRS}
)

add_subdirectory(test_push_recovery_mujoco)
