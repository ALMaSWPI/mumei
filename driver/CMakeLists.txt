project(huron_driver)

add_subdirectory(can)
add_subdirectory(config)

add_library(huron_driver
    INTERFACE
    can
    config)
add_library(huron::driver ALIAS huron_driver)

# Expose the include files
# set(huron_driver_INCLUDE_DIRS
#     ${huron_driver_can_INCLUDE_DIRS}
#     ${huron_driver_config_INCLUDE_DIRS}
# )
# target_include_directories(huron_driver PUBLIC ${huron_driver_INCLUDE_DIRS})

