# File:   FindHDF5.cmake
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
# Find an HDF5 installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: HDF5_FOUND        - TRUE, if the package has been completely found
#   Variable: HDF5_INCLUDE_DIRS - List of full paths to include directories required for using HDF5
#   Variable: HDF5_LIBRARIES    - List of full paths to libraries requiref for using HDF5
#   Function: USE_HDF5()      - Convenience function for preparing CMake for usage of HDF5.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for HDF5: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( HDF5_DIR "" CACHE PATH "Path to an HDF5 installation" )

MESSAGE( STATUS "Looking for HDF5: include directory" )
FIND_PATH( HDF5_INCLUDE_DIR hdf5.h
	PATHS
		${HDF5_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE
)
MESSAGE( STATUS "Looking for HDF5: include directory (${HDF5_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for HDF5: base library" )
FIND_LIBRARY( HDF5_BASE_LIBRARY
	NAMES hdf5
	PATHS
		${HDF5_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for HDF5: base library (${HDF5_BASE_LIBRARY})" )

MESSAGE( STATUS "Looking for HDF5: base library (cpp)" )
FIND_LIBRARY( HDF5_BASE_LIBRARY_CPP
	NAMES hdf5_cpp
	PATHS
		${HDF5_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for HDF5: base library (cpp) (${HDF5_BASE_LIBRARY_CPP})" )

MESSAGE( STATUS "Looking for HDF5: highlevel library" )
FIND_LIBRARY( HDF5_HIGHLEVEL_LIBRARY
	NAMES "hdf5_hl"
	PATHS
		${HDF5_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for HDF5: highlevel library (${HDF5_HIGHLEVEL_LIBRARY})" )

MESSAGE( STATUS "Looking for HDF5: highlevel library (cpp)" )
FIND_LIBRARY( HDF5_HIGHLEVEL_LIBRARY_CPP
	NAMES hdf5_hl_cpp
	PATHS
		${HDF5_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for HDF5: highlevel library (cpp) (${HDF5_HIGHLEVEL_LIBRARY_CPP})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( HDF5 DEFAULT_MSG
	HDF5_BASE_LIBRARY
	HDF5_BASE_LIBRARY_CPP
	HDF5_HIGHLEVEL_LIBRARY
	HDF5_HIGHLEVEL_LIBRARY_CPP
	HDF5_INCLUDE_DIR
)

IF( HDF5_FOUND )
	SET( HDF5_INCLUDE_DIRS "${HDF5_INCLUDE_DIR}")
	SET (HDF5_LIBRARIES
		"${HDF5_BASE_LIBRARY}"
		"${HDF5_BASE_LIBRARY_CPP}"
		"${HDF5_HIGHLEVEL_LIBRARY}"
		"${HDF5_HIGHLEVEL_LIBRARY_CPP}"
	)

	# Function making HDF5 ready to be used.
	FUNCTION( USE_HDF5 )
		IF( NOT HDF5_USED )
			INCLUDE_DIRECTORIES( ${HDF5_INCLUDE_DIRS} )
			SET( HDF5_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using HDF5 in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_HDF5 )

ENDIF()

IF( HDF5_FOUND OR HDF5_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		HDF5_DIR
		HDF5_INCLUDE_DIR
		HDF5_BASE_LIBRARY
		HDF5_BASE_LIBRARY_CPP
		HDF5_HIGHLEVEL_LIBRARY
		HDF5_HIGHLEVEL_LIBRARY_CPP
	)
ENDIF()
