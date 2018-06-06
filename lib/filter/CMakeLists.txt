cmake_minimum_required(VERSION 2.6)
project(filter)

set(FILTER_MAJOR_VERSION "1")
set(FILTER_MINOR_VERSION "0")
set(FILTER_PATCH_VERSION "0")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Set the C99 standard for the C files
set(CMAKE_C_FLAGS --std=gnu99)
add_definitions(-fPIC)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(ach somatic amino)

# Compile the fortran file
add_custom_command(
	OUTPUT kalman.o
	COMMAND gfortran -c ../kalman.f90 -fPIC
)

# Add the library
add_library(filter SHARED filter.c kalman.o)

# Install it
install(FILES filter.h DESTINATION /usr/local/include)
install(TARGETS filter DESTINATION /usr/local/lib)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "filter")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${FILTER_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${FILTER_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${FILTER_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libach-dev, somatic, libamino-dev")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Neil Dantam <ntd@gatech.edu>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "This library contains filtering functions such as kalman and particle filters")
set(CPACK_PACKAGE_VENDOR "Georgia Tech Humanoids Lab")
include(CPack)
