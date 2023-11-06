project(test_robot)

add_executable(test_robot_api test_robot_api.cc)

target_link_libraries(test_robot_api
        PUBLIC
        sockcanpp
        huron
)

