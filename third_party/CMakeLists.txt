project(huron_third_party)

add_subdirectory(libsockcanpp)

add_library(huron_third_party INTERFACE libsockcanpp)

set(libsockcanpp_INCLUDE_DIRS
    libsockcanpp/include
    libsockcanpp/include/exceptions)
set(huron_third_party_INCLUDE_DIRS
    ${libsockcanpp_INCLUDE_DIRS})
target_include_directories(huron_third_party
    INTERFACE 
    $<BUILD_INTERFACE:${huron_third_party_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>)

