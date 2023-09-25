project(huron_system_exceptions)

# Add a library
add_library(${PROJECT_NAME} INTERFACE)
add_library(huron::system::exceptions ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
  INTERFACE ${PROJECT_SOURCE_DIR}/include
)
