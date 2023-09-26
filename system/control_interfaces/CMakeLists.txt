project(huron_system_ci)

# Add a library
add_library(${PROJECT_NAME} src/joint.cc src/revolute_joint.cc src/limb.cc src/robot.cc)
add_library(huron::system::ci ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        huron::utils
)
