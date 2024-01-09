cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)

project(test_build_from_yaml)

add_executable(test_build_from_yaml build_robot_from_yaml.cc)

target_link_libraries(test_build_from_yaml
    PUBLIC
        huron
)
#target_include_directories(build_robot_from_yaml
#    PUBLIC
#    ${utils_INCLUDE_DIRS}/..)
set(dir "${CMAKE_CURRENT_SOURCE_DIR}")