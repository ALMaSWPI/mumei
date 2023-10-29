project(huron_system_exceptions)

# Add a library
add_library(${PROJECT_NAME} INTERFACE)
add_library(huron::system::exceptions ALIAS ${PROJECT_NAME})

set(huron_system_exceptions_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(huron_system_exceptions
    INTERFACE ${huron_system_exceptions_INCLUDE_DIRS}
)
install(DIRECTORY ${huron_system_exceptions_INCLUDE_DIRS} DESTINATION include)
