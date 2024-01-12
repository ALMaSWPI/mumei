cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(test_robot_api)

find_package(huron REQUIRED)

add_executable(test_robot_api test_robot_api.cc)

target_link_libraries(test_robot_api
        PUBLIC
        huron::huron
)

