# File:   BuildPaths.cmake
# Author: Christian Hoffmann
# Date:   2007--2008
#
# This file is part of the MUSCOD/VPLAN suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2007--2008. All rights reserved.
#
####################################################################################################
#
# Define variables with install/build paths
#
####################################################################################################

IF( NOT _BUILDPATHS_ )
SET( _BUILDPATHS_ TRUE )

IF( NOT IS_DIRECTORY "${PROJECT_BINARY_DIR}" )
	ERROR( "Failed setting build dirs, project binary dir '${PROJECT_BINARY_DIR}' invalid." )
ENDIF()
IF( NOT IS_DIRECTORY "${CMAKE_INSTALL_PREFIX}" )
	ERROR( "Failed setting build dirs, install dir '${CMAKE_INSTALL_PREFIX}' invalid." )
ENDIF()


# Paths where files generated at configuration time are stored
SET( TEMP_INCLUDE_DIR    "${PROJECT_BINARY_DIR}/temp/include" )
SET( TEMP_INTERFACE_DIR  "${PROJECT_BINARY_DIR}/temp/interfaces" )
IF( BUILD_32BIT )
	SET( TEMP_LIBRARY_DIR    "${PROJECT_BINARY_DIR}/temp/lib" )
ELSE( BUILD_32BIT )
	SET( TEMP_LIBRARY_DIR    "${PROJECT_BINARY_DIR}/temp/lib64" )
ENDIF()
SET( TEMP_EXECUTABLE_DIR "${PROJECT_BINARY_DIR}/temp/bin" )
SET( TEMP_TEST_DIR       "${PROJECT_BINARY_DIR}/temp/test" )
SET( TEMP_DOC_DIR        "${PROJECT_BINARY_DIR}/temp/doc" )
SET( TEMP_AUX_DIR        "${PROJECT_BINARY_DIR}/temp/aux" )
SET( TEMP_CMAKE_DIR      "${PROJECT_BINARY_DIR}/temp/cmake" )

# Default install paths. INSTALL_XXX paths are relative to CMAKE_INSTALL_PREFIX
SET( INSTALL_EXECUTABLE_DIR "${CMAKE_INSTALL_PREFIX}/bin" )
SET( INSTALL_INCLUDE_DIR    "${CMAKE_INSTALL_PREFIX}/include" )
IF( BUILD_32BIT )
	SET( INSTALL_LIBRARY_DIR    "${CMAKE_INSTALL_PREFIX}/lib" )
ELSE( BUILD_32BIT )
	SET( INSTALL_LIBRARY_DIR    "${CMAKE_INSTALL_PREFIX}/lib64" )
ENDIF()
SET( INSTALL_PACKAGE_DIR    "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}" )
SET( INSTALL_INTERFACE_DIR  "${INSTALL_PACKAGE_DIR}/interfaces" )
SET( INSTALL_DOC_DIR        "${INSTALL_PACKAGE_DIR}/doc" )
SET( INSTALL_AUX_DIR        "${INSTALL_PACKAGE_DIR}/aux" )
SET( INSTALL_CMAKE_DIR      "${INSTALL_PACKAGE_DIR}/cmake" )
SET( INSTALL_EXAMPLE_DIR    "${INSTALL_PACKAGE_DIR}/examples" )
SET( INSTALL_TEST_DIR       "${INSTALL_PACKAGE_DIR}/test" )

ENDIF()
