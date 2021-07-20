# File:   FindPLPLOT.cmake
# Author: Jan Albersmeyer
# Date:   2009
#
# Copyright (C) 2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Find an PLPLOT installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: PLPLOT_FOUND        - TRUE, if the package has been completely found
#   Variable: PLPLOT_INCLUDE_DIRS - List of full paths to include directories required for using PLPLOT
#   Variable: PLPLOT_LIBRARIES    - List of full paths to libraries requiref for using PLPLOT
#   Function: USE_PLPLOT()      - Convenience function for preparing CMake for usage of PLPLOT.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for PLPLOT: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( PLPLOT_DIR "" CACHE PATH "Path to an PLPLOT installation" )

MESSAGE( STATUS "Looking for PLPLOT: include directory" )
FIND_PATH( PLPLOT_INCLUDE_DIR plplot.h
	PATHS
		${PLPLOT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE plplot
)

MESSAGE( STATUS "Looking for PLPLOT: include directory (${PLPLOT_INCLUDE_DIR})" )


MESSAGE( STATUS "Looking for PLPLOT: core library" )
FIND_LIBRARY( PLPLOT_CORE_LIBRARY
	NAMES plplotd
	PATHS
		${PLPLOT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for PLPLOT: core library (${PLPLOT_CORE_LIBRARY})" )

MESSAGE( STATUS "Looking for PLPLOT: cxx bindings library" )
FIND_LIBRARY( PLPLOT_CXX_BINDINGS_LIBRARY
	NAMES plplotcxxd
	PATHS
		${PLPLOT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for PLPLOT: cxx bindings library (${PLPLOT_CXX_BINDINGS_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	PLPLOT
	DEFAULT_MSG
	PLPLOT_CORE_LIBRARY
	PLPLOT_CXX_BINDINGS_LIBRARY
	PLPLOT_INCLUDE_DIR
	)

IF( PLPLOT_FOUND )
	SET( INCLUDE_BASE "${PLPLOT_INCLUDE_DIR}/../")

	SET( PLPLOT_INCLUDE_DIRS
		"${PLPLOT_INCLUDE_DIR}"
		)

	SET( PLPLOT_LIBRARIES
		"${PLPLOT_CORE_LIBRARY}"
		"${PLPLOT_CXX_BINDINGS_LIBRARY}"
	)

	# Function making PLPLOT ready to be used.
	FUNCTION( USE_PLPLOT )
		IF( NOT PLPLOT_USED )
			INCLUDE_DIRECTORIES( ${PLPLOT_INCLUDE_DIRS} )
			SET( PLPLOT_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using PLPLOT in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_PLPLOT )

ENDIF()

IF( PLPLOT_FOUND OR PLPLOT_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		PLPLOT_DIR
		PLPLOT_INCLUDE_DIR
		PLPLOT_CORE_LIBRARY
		PLPLOT_CXX_BINDINGS_LIBRARY
		)
ENDIF()
