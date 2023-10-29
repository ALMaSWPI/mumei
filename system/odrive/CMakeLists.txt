project(huron_system_odrive)


add_library(${PROJECT_NAME}
        src/odrive.cc
        src/odrive_can.cc
        src/odrive_rotary_encoder.cc
        src/odrive_torque_motor.cc)
add_library(huron::system::odrive ALIAS ${PROJECT_NAME})

set(huron_system_odrive_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(huron_system_odrive
    PUBLIC ${huron_system_odrive_INCLUDE_DIRS}
)

target_link_libraries(huron_system_odrive
        PUBLIC
        huron::system::ci
        huron::system::exceptions
        huron::driver::can
        huron::utils
)

install(DIRECTORY ${huron_system_odrive_INCLUDE_DIRS} DESTINATION include)

add_subdirectory(test)
