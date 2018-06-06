# @author Can Erdogan
# @date June 28, 2013
# @brief This file is to be included in a cmake to avoid cluttering it with cpack calls
# that would be mostly the same across different cmakes.
# The user is responsible of setting CPACK_PACKAGE_VERSION, CPACK_DEBIAN_PACKAGE_DEPENDS,
# and CPACK_PACKAGE_DESCRIPTION_SUMMARY/ 
# NOTE I have put my name under package maintainer as I'd be the responsible of all packages
# concerning Krang software.

# Set the package name and the system properties
set(CPACK_PACKAGE_NAME ${CPACK_PROJECT_NAME})
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()

# Set the package file name, maintainer and the vendor
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Can Erdogan <cerdogan3@gatech.edu>")
set(CPACK_PACKAGE_VENDOR "Georgia Tech Humanoids Lab")

# Include CPack to actually create the package
include(CPack)
