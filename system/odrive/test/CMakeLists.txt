project (test_system_odrive)

add_executable(test_encoder test_encoder.cc)

target_link_libraries(test_encoder
        PUBLIC
        huron::system::odrive
        huron::driver::config
        huron::system::ci
)

