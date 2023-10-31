project (test_odrive_can)

add_executable(move_motor move_motor.cc)
add_executable(read_encoder read_encoder.cc)
add_executable(read_encoder_est read_encoder_est.cc)

target_link_libraries(move_motor
    PUBLIC
    huron
)

target_link_libraries(read_encoder
    PUBLIC
    huron
)

target_link_libraries(read_encoder_est
    PUBLIC
    huron
)
