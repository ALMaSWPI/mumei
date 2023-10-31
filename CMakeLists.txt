cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

project(huron)

# Find sockcanpp
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(sockcanpp REQUIRED)
message(sockcanpp_INCLUDE_DIRS="${sockcanpp_INCLUDE_DIRS}")

add_library(huron SHARED huron.cc)
set_target_properties(huron PROPERTIES LINKER_LANGUAGE CXX)

# Add subpackages
add_subdirectory(driver)
add_subdirectory(utils)
add_subdirectory(system)
# add_subdirectory(third_party)

target_link_libraries(huron
    PUBLIC
    sockcanpp
)
target_include_directories(huron
    PUBLIC
    ${sockcanpp_INCLUDE_DIRS}/..)

# Install the "huronConfig.cmake" file and export the target
install(TARGETS huron
  EXPORT huronTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  # FILE_SET HEADERS DESTINATION include
)

# Generate and install the export targets for "huron"
install(EXPORT huronTargets
  FILE huronTargets.cmake
  NAMESPACE huron::
  DESTINATION lib/cmake/huron
)

# Generate and install huronConfig.cmake
include(CMakePackageConfigHelpers)

# Generate ${PROJECT_NAME}Config.cmake
configure_package_config_file(
  huronConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/huronConfig.cmake
  INSTALL_DESTINATION lib/cmake/huron
)
# Install the generated "huronConfig.cmake" file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/huronConfig.cmake
    DESTINATION lib/cmake/huron
)
# Also install Findsockcanpp.cmake
install(
    FILES Findsockcanpp.cmake
    DESTINATION lib/cmake/huron/Modules
)

# Build examples
add_subdirectory(examples)
