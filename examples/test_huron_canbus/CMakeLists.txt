project (test_huron_canbus)

add_executable(send_msg send_msg.cc)
add_executable(read_msg read_msg.cc)

target_link_libraries(send_msg
    PUBLIC
        huron_driver::can
)

target_link_libraries(read_msg
    PUBLIC
        huron_driver::can
)
