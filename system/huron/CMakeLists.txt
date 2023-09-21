project(huron_system_huron)

add_library(${PROJECT_NAME} src/huron_encoder.cc src/torque_encoder.cc)
add_library(huron::system::huron ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        huron::system::ci
        huron::driver::can
        huron::utils
)
