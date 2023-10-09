project(huron_system)

add_subdirectory(control_interfaces)
add_subdirectory(exceptions)
add_subdirectory(odrive)

add_library(${PROJECT_NAME}
    INTERFACE 
    control_interfaces
    exceptions
    odrive)
add_library(huron::system ALIAS ${PROJECT_NAME})
