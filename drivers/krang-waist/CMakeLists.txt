cmake_minimum_required(VERSION 2.6)
project(krangWaist)

set(KRANG_WAIST_MAJOR_VERSION "1")
set(KRANG_WAIST_MINOR_VERSION "1")
set(KRANG_WAIST_PATCH_VERSION "0")

# Set the C99 standard for the C files
set(CMAKE_C_FLAGS --std=gnu99)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either
# in /usr/lib or /usr/local/lib
link_libraries(ach somatic amino protobuf-c)

# Add the executables
include_directories(include)
add_executable(krang-waist krang-waist.cpp)

# Install it
install(TARGETS krang-waist DESTINATION /usr/local/bin)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "krang-waist")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${KRANG_WAIST_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${KRANG_WAIST_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${KRANG_WAIST_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libach-dev, somatic, amciod, libamino-dev")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Can Erdogan <cerdogan3@gatech.edu>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "An interface to control the Schunk modules using the somatic library")
set(CPACK_PACKAGE_VENDOR "Georgia Tech Humanoids Lab")
include(CPack)
