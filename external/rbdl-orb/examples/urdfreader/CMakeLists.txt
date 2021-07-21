PROJECT (RBDLEXAMPLE CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )


SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

# Search for the RBDL include directory and library
FIND_PACKAGE (RBDL COMPONENTS URDFREADER REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# Create an executable
ADD_EXECUTABLE (example_urdfreader example_urdfreader.cc)

# And link the library against the executable
TARGET_LINK_LIBRARIES ( example_urdfreader
	${RBDL_LIBRARY}
	${RBDL_URDFREADER_LIBRARY}
	)
