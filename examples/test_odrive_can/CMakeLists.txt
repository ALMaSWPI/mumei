project (test_odrive_can)

add_executable(move_motor move_motor.cc)
# add_executable(send_sockcanpp receive_sockcanpp.cc)

target_link_libraries(move_motor
    PUBLIC
        huron::driver::can
        huron::driver::config
)
