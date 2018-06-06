# @author Can Erdogan
# @date Jan 15, 2012

cmake_minimum_required(VERSION 2.6)
project(forwardKinematics)

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_CXX_FLAGS -g)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

#Include the OpenCV library
#set(OpenCV_DIr "/usr/share/OpenCV")
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIR})
#link_libraries(${OpenCV_LIBS})
#
##Include the PCL Library
#find_package(PCL 1.6 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})

# Include Dart and Grip stuff
include(simulation.cmake)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(amino ntcan protobuf-c ach somatic stdc++)

# Include Eigen
include_directories(/usr/include/eigen3)
include_directories(/usr/local/include/eigen3)

# Compile the main executable
add_executable(sensors sensors.cpp ssdmu.c)
target_link_libraries(sensors ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
target_link_libraries(sensors ${PCL_LIBRARIES} )
add_custom_target(sensors.run sensors ${ARGN})
