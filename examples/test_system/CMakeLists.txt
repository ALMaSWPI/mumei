project (test_system)

add_executable(test_system_api test_system_api.cc)

target_link_libraries(test_system_api
        PUBLIC
        huron
)

