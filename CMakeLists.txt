cmake_minimum_required(VERSION 3.5)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

# Configure target build platform. Defaults to building for Raspberry Pi (armv7l).
set(BUILD_TYPE "armv7l" CACHE STRING "Choose the build type: x86_64 or armv7l")

    # Conditionally set the toolchain file based on the build type
if(BUILD_TYPE STREQUAL "armv7l")
    set(CMAKE_TOOLCHAIN_FILE "tools/armhf-toolchain.cmake")
elseif(BUILD_TYPE STREQUAL "x86_64")
    set(CMAKE_TOOLCHAIN_FILE "tools/x86_64-toolchain.cmake")
else()
    message(FATAL_ERROR "Invalid build type. Use -DBUILD_TYPE=x86_64 or -DBUILD_TYPE=armv7l")
endif()

project (huron)

add_subdirectory(examples)
add_subdirectory(driver)
add_subdirectory(utils)
add_subdirectory(system)
add_subdirectory(third_party/libsockcanpp)

