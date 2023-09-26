project(huron_system_odrive)


add_library(${PROJECT_NAME}
        src/odrive_can.cc
        src/odrive_rotary_encoder.cc
        src/odrive_torque_motor.cc)
add_library(huron::system::odrive ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        huron::system::ci
        huron::system::exceptions
        huron::driver::can
        huron::utils
)

add_subdirectory(test)
