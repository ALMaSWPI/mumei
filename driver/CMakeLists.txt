project(huron_driver)

add_subdirectory(can)
add_subdirectory(config)

add_library(${PROJECT_NAME}
    INTERFACE
    can
    config)
add_library(huron::driver ALIAS ${PROJECT_NAME})

# Include directories for the "driver" subproject
# target_include_directories(${PROJECT_NAME} PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/can/include/huron_driver
#     ${CMAKE_CURRENT_SOURCE_DIR}/config/include/huron_driver
# )

# Link your "driver" subproject against the "can" library
# target_link_libraries(${PROJECT_NAME}
#     PUBLIC huron_driver::can huron_driver::config)
