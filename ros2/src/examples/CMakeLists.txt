cmake_minimum_required(VERSION 3.8)
project(examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../build")
find_package(huron_ros2 REQUIRED)
#if(huron_ros2)
#  target_link_libraries(examples PRIVATE huron_ros2)
#  target_include_directories(${PROJECT_NAME} PUBLIC
#          $<BUILD_INTERFACE:${huron_ros2_INCLUDE_DIRS} include>
#          $<INSTALL_INTERFACE:include>)
#else()
#  message(FATAL_ERROR "huron_ros2 not found.")
#endif()
#include(huron_ros2 REQUIRED)

add_executable(test1 src/test_main.cc)
ament_target_dependencies(test1 rclcpp )
target_link_libraries(test1 huron_ros2::huron_ros2)

install(TARGETS
        test1
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
