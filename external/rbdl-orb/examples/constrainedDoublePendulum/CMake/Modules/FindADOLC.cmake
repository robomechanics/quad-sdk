# File:   FindADOLC.cmake
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
# Find an ADOL-C installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: ADOLC_FOUND        - TRUE, if the package has been completely found
#   Variable: ADOLC_INCLUDE_DIRS - List of full paths to include directories required for using ADOLC
#   Variable: ADOLC_LIBRARIES    - List of full paths to libraries requiref for using ADOLC
#   Function: USE_ADOLC()      - Convenience function for preparing CMake for usage of ADOLC.
#
# TODO: what the heck is ColPack?
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for ADOLC: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( ADOLC_DIR "" CACHE PATH "Path to an ADOL-C installation" )

MESSAGE( STATUS "Looking for ADOLC: include directory" )
FIND_PATH( ADOLC_INCLUDE_DIR adolc.h
	PATHS
		${ADOLC_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE adolc_base/include/adolc
)

MESSAGE( STATUS "Looking for ADOLC: include directory (${ADOLC_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for ADOLC: library" )
FIND_LIBRARY( ADOLC_LIBRARY
	NAMES adolc
	PATHS
		${ADOLC_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB adolc_base/lib 
)
MESSAGE( STATUS "Looking for ADOLC: library (${ADOLC_LIBRARY})" )

MESSAGE( STATUS "Looking for ColPack: library" )
FIND_LIBRARY( COLPACK_LIBRARY
	NAMES ColPack
	PATHS
		${ADOLC_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB adolc_base/lib
)
MESSAGE( STATUS "Looking for ColPack: library (${COLPACK_LIBRARY})" )

# FIXME: hack for MUSCOD apps that use muscod's adcaux library. Required as MUSCOD currently provides
# no consistent way for its apps to determine what the exact path to a module library is.
# Should be removed as soon as a proper package interaction functionality has been (re-)implemented
# in the CMake framework.
MESSAGE( STATUS "Looking for optional MUSCOD's adcaux library" )
FIND_LIBRARY( MUSCOD_ADCAUX_LIBRARY
	NAMES muscod_adcaux
	PATHS
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES  lib64 lib
)
MESSAGE( STATUS "Looking for optional MUSCOD's adcaux library (${MUSCOD_ADCAUX_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	ADOLC
	DEFAULT_MSG
	ADOLC_LIBRARY
 	COLPACK_LIBRARY
	ADOLC_INCLUDE_DIR
)

IF( ADOLC_FOUND )
	SET( ADOLC_INCLUDE_DIRS "${ADOLC_INCLUDE_DIR}" "${ADOLC_INCLUDE_DIR}/../" )
	SET( ADOLC_LIBRARIES
		"${ADOLC_LIBRARY}"
 		"${COLPACK_LIBRARY}"
	)

	# Function making ADOLC ready to be used.
	FUNCTION( USE_ADOLC )
		IF( NOT ADOLC_USED )
			INCLUDE_DIRECTORIES( ${ADOLC_INCLUDE_DIRS} )
			SET( ADOLC_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using ADOLC in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_ADOLC )

ENDIF()

IF( ADOLC_FOUND OR ADOLC_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		ADOLC_DIR
		ADOLC_INCLUDE_DIR
		ADOLC_LIBRARY
 		COLPACK_LIBRARY
	)
ENDIF()
