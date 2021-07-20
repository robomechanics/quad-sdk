# File:   FindXERCESC.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of proprietary software of the
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Find a XERCES-C installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: XERCESC_FOUND        - TRUE, if the package has been completely found
#   Variable: XERCESC_INCLUDE_DIRS - List of full paths to include directories required for using XERCESC
#   Variable: XERCESC_LIBRARIES    - List of full paths to libraries required for using XERCESC
#   Function: USE_XERCESC()      - Convenience function for preparing CMake for usage of XERCESC.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for XERCESC: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( XERCESC_DIR "" CACHE PATH "Path to a XERCESC installation" )

MESSAGE( STATUS "Looking for XERCESC: library" )
FIND_LIBRARY( XERCESC_LIBRARY
	NAMES xerces-c
	PATHS
		${XERCESC_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
#   NO_DEFAULT_PATH
#   NO_CMAKE_ENVIRONMENT_PATH
#   NO_CMAKE_PATH
#   NO_SYSTEM_ENVIRONMENT_PATH
#   NO_CMAKE_SYSTEM_PATH
)
MESSAGE( STATUS "Looking for XERCESC: library (${XERCESC_LIBRARY})" )

MESSAGE( STATUS "Looking for XERCESC: include dir" )
FIND_PATH( XERCESC_INCLUDE_DIR
	NAMES XercesDefs.hpp
	PATHS
		${XERCESC_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES include/xercesc/util xercesc/util
#   NO_DEFAULT_PATH
#   NO_CMAKE_ENVIRONMENT_PATH
#   NO_CMAKE_PATH
#   NO_SYSTEM_ENVIRONMENT_PATH
#   NO_CMAKE_SYSTEM_PATH
)
MESSAGE( STATUS "Looking for XERCESC: include dir (${XERCESC_INCLUDE_DIR})" )

# IF( XERCESC_INCLUDE_DIR )
# 	STRING( REGEX REPLACE "xercesc/util: " TEMP "${XERCESC_INCLUDE_DIR}" )
# 	SET( XERCESC_INCLUDE_DIR "${TEMP}" CACHE PATH "" FORCE )
# ENDIF()


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( XERCESC DEFAULT_MSG
	XERCESC_LIBRARY
	XERCESC_INCLUDE_DIR
)

IF( XERCESC_FOUND )
	SET( XERCESC_LIBRARIES "${XERCESC_LIBRARY}" )
	SET( XERCESC_INCLUDE_DIRS "${XERCESC_INCLUDE_DIR}" )

	# Function making XERCESC ready to be used.
	FUNCTION( USE_XERCESC )
		IF( NOT XERCESC_USED )
			INCLUDE_DIRECTORIES( ${XERCESC_INCLUDE_DIRS} )
			SET( XERCESC_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using XERCESC in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_XERCESC )

ENDIF()

IF( XERCESC_FOUND OR XERCES_FIND_QUIETLY )
	MARK_AS_ADVANCED (
		XERCESC_DIR
		XERCESC_LIBRARY
		XERCESC_INCLUDE_DIR
	)
ENDIF()
