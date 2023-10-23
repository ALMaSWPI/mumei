project (test_system_odrive)

add_executable(test_encoder test_encoder.cc)

target_link_libraries(test_encoder
        PUBLIC
        huron::system::odrive
        huron::driver::config
        huron::system::ci
)

add_executable(test_torque_motor test_torque_motor.cc)

target_link_libraries(test_torque_motor
        PUBLIC
        huron::system::odrive
        huron::driver::config
        huron::system::ci
)

add_executable(test_revolute_joint test_revolute_joint.cc)

target_link_libraries(test_revolute_joint
        PUBLIC
        huron::system::odrive
        huron::driver::config
        huron::system::ci
)

add_executable(test_revolute_joint_encoder test_revolute_joint_encoder.cc)

target_link_libraries(test_revolute_joint_encoder
        PUBLIC
        huron::system::odrive
        huron::driver::config
        huron::system::ci
)

add_executable(test_configure_limits test_configure_limits.cc)

target_link_libraries(test_configure_limits
        PUBLIC
        huron::system::odrive
        huron::driver::config
        huron::system::ci
)
