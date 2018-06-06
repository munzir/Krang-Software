# @author Saul Reynolds-Haertle
# @date 7 Aug 2013

# set up the project
cmake_minimum_required(VERSION 2.6)
project(kore)

# use c++0x
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=gnu++0x")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Include Dart, Grip, and Eigen
include(cmake/simulation.cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)
include_directories(/usr/include/eigen3)

# link to various libraries that we need
link_libraries(protobuf-c amino ach somatic filter stdc++ ncurses)

# set our include directories
include_directories(include)

# build the kore library
add_library(kore SHARED src/display.cpp src/kore.cpp src/safety.cpp src/sensors.cpp src/util.cpp src/workspace.cpp
		src/ik.cpp)

# and add our install target
install(TARGETS kore DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
file(GLOB headers "include/*.hpp")
file(GLOB headers-kore "include/kore/*.hpp")
install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(FILES ${headers-kore} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/kore)

# Create a specific target to install the configuration files for the Krang and object model
add_custom_target(install-config
	COMMAND rm -rf /etc/kore
	COMMAND mkdir /etc/kore 
	COMMAND cp -r ../etc/model /etc/kore/
	COMMAND cp -r ../etc/objects /etc/kore/
	COMMAND cp -r ../etc/scenes /etc/kore/
)

# set up cpack
set(CPACK_PACKAGE_NAME "kore")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 0)
set(CPACK_PACKAGE_VERSION_PATCH 3)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libach-dev, libamino-dev, libncurses5")

set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Can Erdogan <cerdogan3@gatech.edu>")

set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Kore is a library for dealing with core functions on Golem Krang")
set(CPACK_PACKAGE_VENDOR "Georgia Tech Humanoids Lab")

include(CPack)

