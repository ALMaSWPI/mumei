project (test_huron_canbus)

add_executable(send_msg send_msg.cc)
add_executable(read_msg read_msg.cc)
add_executable(msg_conversion msg_conversion.cc)

target_link_libraries(send_msg
    PUBLIC
        huron::driver::can
        huron::driver::config
        huron::system::odrive
)

target_link_libraries(read_msg
    PUBLIC
        huron::driver::can
        huron::utils
)

target_link_libraries(msg_conversion
    PUBLIC
        huron::driver::can
        huron::utils
        huron::system::odrive
)
