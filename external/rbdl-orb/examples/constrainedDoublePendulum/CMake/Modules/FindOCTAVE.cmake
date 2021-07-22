# File:   FindOCTAVE.cmake
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
####################################################################################################
#
# Find an OCTAVE installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: OCTAVE_FOUND        - TRUE, if the package has been completely found
#   Variable: OCTAVE_INCLUDE_DIRS - List of full paths to include directories required for using OCTAVE
#   Variable: OCTAVE_LIBRARIES    - List of full paths to libraries required for using OCTAVE
#   Variable: OCTAVE_OCT_LINK_LIBRARIES    - List of libraries required linking an oct file [cached]
#   Function: USE_OCTAVE()      - Convenience function for preparing CMake for usage of OCTAVE.
#
# TODO: hard-coded linker string in OCTAVE_OCT_LINK_LIBRARIES
# TODO: hard-coded gfortran-dependency in OCTAVE_OCT_LINK_LIBRARIES
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for OCTAVE: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( OCTAVE_DIR "" CACHE PATH "Path to an OCTAVE installation" )

MESSAGE( STATUS "Looking for OCTAVE: include directory" )
FIND_PATH( OCTAVE_INCLUDE_DIR octave.h
	PATHS
		${OCTAVE_DIR}
		/usr/include/octave-3.0.2/octave
		/usr/include/octave-3.0.2/
		/usr/include/octave
		/usr/local/include/octave
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE
)
MESSAGE( STATUS "Looking for OCTAVE: include directory (${OCTAVE_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for OCTAVE: base library" )
FIND_LIBRARY( OCTAVE_BASE_LIBRARY
	NAMES octave
	PATHS
		${OCTAVE_DIR}
		/usr/lib64/octave-3.0.2
		/usr/lib/octave
		/usr/local/lib/octave
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for OCTAVE: base library (${OCTAVE_BASE_LIBRARY})" )

MESSAGE( STATUS "Looking for OCTAVE: interp library" )
FIND_LIBRARY( OCTAVE_INTERP_LIBRARY
	NAMES octinterp
	PATHS
		${OCTAVE_DIR}
		/usr/lib64/octave-3.0.2
		/usr/lib/octave
		/usr/local/lib/octave
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for OCTAVE: interp library (${OCTAVE_INTERP_LIBRARY})" )

MESSAGE( STATUS "Looking for OCTAVE: cruft" )
FIND_LIBRARY( OCTAVE_CRUFT_LIBRARY
	NAMES cruft
	PATHS
		${OCTAVE_DIR}
		/usr/lib64/octave-3.0.2
		/usr/lib/octave
		/usr/local/lib/octave
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for OCTAVE: cruft (${OCTAVE_CRUFT_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( OCTAVE DEFAULT_MSG
	OCTAVE_BASE_LIBRARY
	OCTAVE_INTERP_LIBRARY
	OCTAVE_INCLUDE_DIR
#	OCTAVE_OCT_LINK_LIBRARIES
)

IF( OCTAVE_FOUND )
	SET( OCTAVE_INCLUDE_DIRS "${OCTAVE_INCLUDE_DIR}")
	SET ( OCTAVE_OCT_LINK_LIBRARIES
	"-llapack -lblas -lreadline -lncurses -ldl -lhdf5 -lz -lm -lhdf5 -lz -lgfortranbegin -lgfortran -lm"
	CACHE STRING "Additional libraries needed to link an oct file"
	)

	SET (OCTAVE_LIBRARIES
		"${OCTAVE_INTERP_LIBRARY}"
		"${OCTAVE_BASE_LIBRARY}"
		"${OCTAVE_CRUFT_LIBRARY}"
		"${OCTAVE_OCT_LINK_LIBRARIES}"
	)

	# Function making OCTAVE ready to be used.
	FUNCTION( USE_OCTAVE )
		IF( NOT OCTAVE_USED )
			INCLUDE_DIRECTORIES( ${OCTAVE_INCLUDE_DIRS} )
			SET( OCTAVE_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using OCTAVE in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_OCTAVE )

ENDIF()

IF( OCTAVE_FOUND OR OCTAVE_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		OCTAVE_DIR
		OCTAVE_INCLUDE_DIR
		OCTAVE_BASE_LIBRARY
		OCTAVE_CRUFT_LIBRARY
		OCTAVE_INTERP_LIBRARY
		OCTAVE_OCT_LINK_LIBRARIES
	)
ENDIF()
