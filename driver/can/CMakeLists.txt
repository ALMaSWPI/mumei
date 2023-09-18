project(huron_driver_can)

# Add a library
add_library(${PROJECT_NAME} src/huron_canbus.cc src/huron_odrive_can.cc)
add_library(huron::driver::can ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        sockcanpp
        huron::utils
)
