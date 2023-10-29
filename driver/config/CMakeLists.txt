project(huron_driver_config)

add_library(huron_driver_config INTERFACE)
add_library(huron::driver::config ALIAS huron_driver_config)

set(huron_driver_config_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(huron_driver_config
    INTERFACE ${huron_driver_config_INCLUDE_DIRS}
)
install(DIRECTORY ${huron_driver_config_INCLUDE_DIRS} DESTINATION include)
