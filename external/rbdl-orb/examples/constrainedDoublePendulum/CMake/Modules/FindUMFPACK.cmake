# File:   FindUMFPACK.cmake
# Author: Jan Albersmeyer
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of proprietary software of the
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
###################################################################################################
#
# Find the UMFPACK includes and libraries
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: UMFPACK_FOUND        - TRUE, if the package has been completely found
#   Variable: UMFPACK_INCLUDE_DIRS - List of full paths to include directories required for using UMFPACK
#   Variable: UMFPACK_LIBRARIES    - List of full paths to libraries required for using UMFPACK
#   Function: USE_UMFPACK()      - Convenience function for preparing CMake for usage of UMFPACK.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for UMFPACK: " )


####################################################################################################
#### SEARCH REQUIRED PACKAGE RESOURCES
####################################################################################################
SET( UMFPACK_DIR "" CACHE PATH "Path to an UMFPACK installation" )

FIND_PACKAGE( BLAS )
FIND_PACKAGE( AMD )

MESSAGE( STATUS "Looking for UMFPACK: include directory" )
FIND_PATH( UMFPACK_INCLUDE_DIR umfpack.h
	PATHS
		${UMFPACK_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES UMFPACK/Include UMFPACK/INCLUDE UMFPACK/include Include INCLUDE include
)
MESSAGE( STATUS "Looking for UMFPACK: include directory (${UMFPACK_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for UMFPACK: library" )
FIND_LIBRARY( UMFPACK_LIBRARY
	NAMES umfpack
	PATHS
		${UMFPACK_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES UMFPACK/Lib UMFPACK/lib UMFPACK/LIB Lib lib  lib64 LIB
)
MESSAGE( STATUS "Looking for UMFPACK: library (${UMFPACK_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( UMFPACK DEFAULT_MSG
	UMFPACK_LIBRARY
	UMFPACK_INCLUDE_DIR
	AMD_FOUND
	BLAS_FOUND
)

IF( UMFPACK_FOUND )
	SET( UMFPACK_INCLUDE_DIRS
		"${AMD_INCLUDE_DIRS}"
		"${UMFPACK_INCLUDE_DIR}"
	)
	SET( UMFPACK_LIBRARIES
		"${UMFPACK_LIBRARY}"
		"${AMD_LIBRARIES}"
		"${BLAS_LIBRARIES}"
	)

	# Function making UMFPACK ready to be used.
	FUNCTION( USE_UMFPACK )
		IF( NOT UMFPACK_USED )
			USE_AMD()
			INCLUDE_DIRECTORIES( ${UMFPACK_INCLUDE_DIRS} )
			SET( UMFPACK_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using UMFPACK in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_UMFPACK )
ENDIF()

IF( UMFPACK_FOUND OR UMFPACK_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		UMFPACK_DIR
		UMFPACK_INCLUDE_DIR
		UMFPACK_LIBRARY
	)
ENDIF()
