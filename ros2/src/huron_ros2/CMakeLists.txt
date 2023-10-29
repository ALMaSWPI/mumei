cmake_minimum_required(VERSION 3.8)
project(huron_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
list(APPEND CMAKE_PREFIX_PATH
        "/usr/local/"
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# huron dependencies
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../build")
# set(huron_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../build")
find_package(huron REQUIRED)
# for debugging
#message(STATUS "huron_INCLUDE_DIRS: ${huron_INCLUDE_DIRS}")
#message(STATUS "huron_LIBRARIES: ${huron_LIBRARIES}")

add_library(${PROJECT_NAME}
  src/revolute_joint.cc
  src/robot.cc
)

# target_link_libraries(${PROJECT_NAME} PUBLIC ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} huron::huron)
target_link_libraries(${PROJECT_NAME} PUBLIC huron::huron)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp sensor_msgs huron)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


# include_directories(include)


ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs huron)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

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
