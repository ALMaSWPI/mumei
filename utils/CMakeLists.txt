project(huron_utils)

add_library(huron_utils INTERFACE)
add_library(huron::utils ALIAS huron_utils)

set(huron_utils_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(huron_utils
    INTERFACE 
    $<BUILD_INTERFACE:${huron_utils_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)

install(DIRECTORY ${huron_utils_INCLUDE_DIRS} DESTINATION include)
