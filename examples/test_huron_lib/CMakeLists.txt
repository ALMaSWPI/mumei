project(test_huron_lib)

find_package(huron REQUIRED)
find_package(sockcanpp REQUIRED)

add_executable(test_huron_lib main.cc)

target_link_libraries(test_huron_lib
    PUBLIC
    huron
    sockcanpp
)

