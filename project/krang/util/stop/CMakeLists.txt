cmake_minimum_required(VERSION 2.6)
project(stop)

# Set the C99 standard for the C files
#set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_CXX_FLAGS -g)

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Link to somatic, amino and ach
link_libraries(amino protobuf-c ach pcio somatic stdc++)

# Create the executable
add_executable(stop stop.cpp)

# Install it
install_targets(/bin stop)
