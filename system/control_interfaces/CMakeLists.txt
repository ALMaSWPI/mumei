project(huron_system_ci)

# Add a library
add_library(${PROJECT_NAME} src/configuration.cc src/joint.cc src/revolute_joint.cc)
add_library(huron::system::ci ALIAS ${PROJECT_NAME})

set(huron_system_ci_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(huron_system_ci
    PUBLIC ${huron_system_ci_INCLUDE_DIRS}
)

target_link_libraries(huron_system_ci
    PUBLIC
        huron::utils
        huron::system::exceptions
)

install(DIRECTORY ${huron_system_ci_INCLUDE_DIRS} DESTINATION include)
