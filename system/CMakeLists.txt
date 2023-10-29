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

# Expose the include files
set(huron_system_INCLUDE_DIRS
    ${huron_system_ci_INCLUDE_DIRS}
    ${huron_system_exceptions_INCLUDE_DIRS}
    ${huron_system_odrive_INCLUDE_DIRS}
)
target_include_directories(huron_system PUBLIC ${huron_system_INCLUDE_DIRS})
