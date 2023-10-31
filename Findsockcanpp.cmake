# Check if the sockcanpp library is available
find_path(sockcanpp_INCLUDE_DIRS NAMES CanDriver.hpp PATHS /usr/local/include/sockcanpp)

# Check if the sockcanpp library is available
find_library(sockcanpp_LIBRARIES NAMES libsockcanpp.so PATHS /usr/local/lib)

# Set sockcanpp_FOUND to TRUE if both the include directory and library are found
if (sockcanpp_INCLUDE_DIRS AND sockcanpp_LIBRARIES)
    set(sockcanpp_FOUND TRUE)
else ()
    set(sockcanpp_FOUND FALSE)
endif ()

# Provide the version of sockcanpp if it's available
find_file(sockcanpp_VERSION_FILE NAMES version.txt PATHS /usr/local/lib/cmake/sockcanpp)
if (sockcanpp_VERSION_FILE)
    file(STRINGS ${sockcanpp_VERSION_FILE} sockcanpp_VERSION)
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(sockcanpp DEFAULT_MSG sockcanpp_LIBRARIES sockcanpp_INCLUDE_DIRS)

if (sockcanpp_FOUND)
    # Include directories for the sockcanpp library
    set(sockcanpp_INCLUDE_DIRS ${sockcanpp_INCLUDE_DIRS})

    # Library file for sockcanpp
    set(sockcanpp_LIBRARIES ${sockcanpp_LIBRARIES})
endif ()
