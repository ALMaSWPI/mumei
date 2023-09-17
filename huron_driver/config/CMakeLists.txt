project(huron_driver_config)

add_library(${PROJECT_NAME} INTERFACE)
add_library(huron_driver::config ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)
