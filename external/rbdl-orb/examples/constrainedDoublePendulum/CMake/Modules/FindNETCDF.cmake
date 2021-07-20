# File:   FindNETCDF.cmake
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
# Find an NETCDF installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: NETCDF_FOUND        - TRUE, if the package has been completely found
#   Variable: NETCDF4_FOUND       - 1, if Version 4 has been found
#   Variable: NETCDF_INCLUDE_DIRS - List of full paths to include directories required for using NETCDF
#   Variable: NETCDF_LIBRARIES    - List of full paths to libraries requiref for using NETCDF
#   Function: USE_NETCDF()      - Convenience function for preparing CMake for usage of NETCDF.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )
INCLUDE( CheckSymbolExists )

MESSAGE( STATUS "Looking for NETCDF: " )

SET( NETCDF_DIR "" CACHE PATH "Path to an NETCDF installation" )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
MESSAGE( STATUS "Looking for NETCDF: include directory" )
FIND_PATH( NETCDF_INCLUDE_DIR netcdf.h
	PATHS
		${NETCDF_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE
)
MESSAGE( STATUS "Looking for NETCDF: include directory (${NETCDF_INCLUDE_DIR})" )

CHECK_SYMBOL_EXISTS( NC_NETCDF4 "${NETCDF_INCLUDE_DIR}/netcdf.h" NETCDF4_FOUND )
MESSAGE( STATUS "Looking for NETCDF: library" )
FIND_LIBRARY( NETCDF_LIBRARY
	NAMES netcdf
	PATHS
		${NETCDF_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for NETCDF: library (${NETCDF_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( NETCDF DEFAULT_MSG NETCDF_LIBRARY NETCDF_INCLUDE_DIR )

IF( NETCDF_FOUND )
	SET( NETCDF_INCLUDE_DIRS "${NETCDF_INCLUDE_DIR}" )
	SET( NETCDF_LIBRARIES "${NETCDF_LIBRARY}" )

	# Function making NETCDF ready to be used.
	FUNCTION( USE_NETCDF )
		IF( NOT NETCDF_USED )
			INCLUDE_DIRECTORIES( ${NETCDF_INCLUDE_DIRS} )
			SET( NETCDF_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using NETCDF in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_NETCDF )

ENDIF()

IF( NETCDF_FOUND OR NETCDF_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		NETCDF_DIR
		NETCDF_INCLUDE_DIR
		NETCDF_LIBRARY
	)
ENDIF()
