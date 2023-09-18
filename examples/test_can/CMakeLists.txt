project (test_can)

add_executable(receive_sockcanpp receive_sockcanpp.cc)
add_executable(send_sockcanpp send_sockcanpp.cc)

target_link_libraries(receive_sockcanpp
    PUBLIC
        sockcanpp
)
target_link_libraries(send_sockcanpp
    PUBLIC
        sockcanpp
)
