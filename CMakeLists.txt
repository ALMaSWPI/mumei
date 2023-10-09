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

project(huron)

add_subdirectory(examples)
add_subdirectory(driver)
add_subdirectory(utils)
add_subdirectory(system)
add_subdirectory(third_party/libsockcanpp)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    system
    driver
    utils)

# Export the configuration package
export(TARGETS ${PROJECT_NAME}
  FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

include(CMakePackageConfigHelpers)

# Generate ${PROJECT_NAME}Config.cmake
configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)
