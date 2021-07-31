# File:   FindSNOPT.cmake
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
# Find an SNOPT installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: SNOPT_FOUND        - TRUE, if the package has been completely found
#   Variable: SNOPT_INCLUDE_DIRS - List of full paths to include directories required for using SNOPT
#   Variable: SNOPT_LIBRARIES    - List of full paths to libraries required for using SNOPT
#   Function: USE_SNOPT()      - Convenience function for preparing CMake for usage of SNOPT.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for SNOPT: " )

####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( SNOPT_DIR "" CACHE PATH "Path to an SNOPT installation" )

MESSAGE( STATUS "Looking for SNOPT: dbl library" )
FIND_LIBRARY(  SNOPT_DBL_LIBRARY
	NAMES dbl
	PATHS
		${SNOPT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib lib64  LIB
)
MESSAGE( STATUS "Looking for SNOPT: dbl library (${SNOPT_DBL_LIBRARY})" )

MESSAGE( STATUS "Looking for SNOPT: opt library" )
FIND_LIBRARY(  SNOPT_OPT_LIBRARY
	NAMES opt
	PATHS
		${SNOPT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib lib64  LIB
)
MESSAGE( STATUS "Looking for SNOPT: opt library (${SNOPT_OPT_LIBRARY})" )

MESSAGE( STATUS "Looking for SNOPT: nopt library" )
FIND_LIBRARY(  SNOPT_NOPT_LIBRARY
	NAMES nopt
	PATHS
		${SNOPT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib lib64  LIB
)
MESSAGE( STATUS "Looking for SNOPT: nopt library (${SNOPT_NOPT_LIBRARY})" )

MESSAGE( STATUS "Looking for SNOPT: snopt library" )
FIND_LIBRARY(  SNOPT_SNOPT_LIBRARY
	NAMES snopt
	PATHS
		${SNOPT_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib lib64  LIB
)
MESSAGE( STATUS "Looking for SNOPT: snopt library (${SNOPT_SNOPT_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( SNOPT DEFAULT_MSG
	SNOPT_DBL_LIBRARY
	SNOPT_OPT_LIBRARY
	SNOPT_NOPT_LIBRARY
	SNOPT_SNOPT_LIBRARY
)

IF( SNOPT_FOUND )
	SET( SNOPT_LIBRARIES
		"${SNOPT_DBL_LIBRARY}"
		"${SNOPT_OPT_LIBRARY}"
		"${SNOPT_NOPT_LIBRARY}"
		"${SNOPT_SNOPT_LIBRARY}"
	)

	# Function making SNOPT ready to be used.
	FUNCTION( USE_SNOPT )
		IF( NOT SNOPT_USED )
			# nothing to do
			SET( SNOPT_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using SNOPT in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_SNOPT )

ENDIF()

IF( SNOPT_FOUND OR SNOPT_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		SNOPT_DIR
		SNOPT_DBL_LIBRARY
		SNOPT_OPT_LIBRARY
		SNOPT_NOPT_LIBRARY
  	SNOPT_SNOPT_LIBRARY
	)
ENDIF()
