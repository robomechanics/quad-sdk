# File:   FindAMD.cmake
# Author: Jab Albersmeyer
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
# Find the AMD includes and libraries.
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: AMD_FOUND        - TRUE, if the package has been completely found
#   Variable: AMD_INCLUDE_DIRS - List of full paths to include directories required for using AMD
#   Variable: AMD_LIBRARIES    - List of full paths to libraries required for using AMD
#   Function: USE_AMD()      - Convenience function for preparing CMake for usage of AMD.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for AMD: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( AMD_DIR "" CACHE PATH "Path to an AMD installation" )

FIND_PACKAGE( UFCONFIG )

MESSAGE( STATUS "Looking for AMD: include directory" )
FIND_PATH( AMD_INCLUDE_DIR amd.h
	PATHS
		${AMD_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES AMD/Include AMD/include AMD/INCLUDE Include include INCLUDE
)
MESSAGE( STATUS "Looking for AMD: include directory (${AMD_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for AMD: library" )
FIND_LIBRARY( AMD_LIBRARY
	NAMES amd
	PATHS
		${AMD_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES AMD/Lib AMD/lib Lib lib  lib64
)
MESSAGE( STATUS "Looking for AMD: library (${AMD_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( AMD DEFAULT_MSG
	AMD_LIBRARY
	AMD_INCLUDE_DIR
	UFCONFIG_FOUND
)

IF( AMD_FOUND )
	SET( AMD_INCLUDE_DIRS "${AMD_INCLUDE_DIR}" "${UFCONFIG_INCLUDE_DIRS}" )
	SET( AMD_LIBRARIES "${AMD_LIBRARY}" "${BLAS_LIBRARIES}" )

	# Function making AMD ready to be used.
	FUNCTION( USE_AMD )
		IF( NOT AMD_USED )
			USE_UFCONFIG()
			INCLUDE_DIRECTORIES( ${AMD_INCLUDE_DIRS} )
			SET( AMD_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using AMD in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_AMD )

ENDIF()

IF( AMD_FOUND OR AMD_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		AMD_DIR
		AMD_INCLUDE_DIR
		AMD_LIBRARY
	)
ENDIF()

