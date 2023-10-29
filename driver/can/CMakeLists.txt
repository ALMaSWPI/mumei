project(huron_driver_can)

# Add a library
add_library(huron_driver_can src/socket_can_bus.cc)
add_library(huron::driver::can ALIAS huron_driver_can)

set(huron_driver_can_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(huron_driver_can
    PUBLIC ${huron_driver_can_INCLUDE_DIRS}
)

target_link_libraries(huron_driver_can
    PUBLIC
        sockcanpp
        huron::utils
)

install(DIRECTORY ${huron_driver_can_INCLUDE_DIRS} DESTINATION include)
