cmake_minimum_required(VERSION 2.6)
project(amciod)

set(AMCIOD_MAJOR_VERSION "1")
set(AMCIOD_MINOR_VERSION "0")
set(AMCIOD_PATCH_VERSION "0")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Set the C99 standard for the C files
set(CMAKE_C_FLAGS --std=gnu99)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(ach somatic ntcan ntcanopen amino)

# Add the executable
include_directories(include)
add_executable(amciod amciod.c amcdrive.c amccan.c)

# Install it
install(TARGETS amciod DESTINATION /usr/local/bin)
install(FILES include/amccan.h include/amcdrive.h include/byteorder.h  DESTINATION /usr/local/include)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "amciod")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${AMCIOD_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${AMCIOD_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${AMCIOD_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libach-dev, somatic, esdcan, libamino-dev")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Can Erdogan <cerdogan3@gatech.edu>")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The amciod daemon allows us to talk with the amcdrives which control Krang's wheels")
set(CPACK_PACKAGE_VENDOR "Georgia Tech Humanoids Lab")

include(CPack)
