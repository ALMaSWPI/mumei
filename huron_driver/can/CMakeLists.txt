project(huron_driver_can)

# Add a library with the above sources
add_library(${PROJECT_NAME} src/huron_canbus.cc)
add_library(huron_driver::can ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        sockcanpp
)
