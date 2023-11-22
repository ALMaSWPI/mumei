# Source: https://github.com/SimonCahill/libsockcanpp
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)

###
# Set binary paths
###
set(tools /usr/bin)
set(cross arm-linux-gnueabihf)

set(CMAKE_C_COMPILER ${tools}/${cross}-gcc)
set(CMAKE_CXX_COMPILER ${tools}/${cross}-g++)
set(CMAKE_LD ${tools}/${cross}-ld)
set(CMAKE_STRIP ${tools}/${cross}-strip)
set(CMAKE_AR ${tools}/${cross}-ar)

set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)
###
# Set source and lib paths
###

#include_directories(SYSTEM )
