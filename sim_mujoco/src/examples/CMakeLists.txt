cmake_minimum_required(VERSION 3.8)
project(examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH
        "/usr/local/"
)

# find dependencies
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(huron_mujoco REQUIRED)
find_package(huron REQUIRED)

add_executable(test_mujoco src/launch_mujoco.cc)
target_link_libraries(test_mujoco
        PUBLIC
        mujoco
        glfw
        huron::huron
)

install(TARGETS
        test_mujoco
        DESTINATION lib/${PROJECT_NAME})



