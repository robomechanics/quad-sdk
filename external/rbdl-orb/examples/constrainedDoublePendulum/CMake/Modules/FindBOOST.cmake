# File:   FindBOOST.cmake
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
# Find a BOOST installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: BOOST_FOUND        - TRUE, if the package has been completely found
#   Variable: BOOST_INCLUDE_DIRS - Full path of the directory containing the BOOST header files
#   Variable: BOOST_LIBRARY_DIRS - Full path of the directory containing the BOOST libraries
#   Function: USE_BOOST()      - Convenience function for preparing CMake for usage of BOOST.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for BOOST: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( BOOST_DIR "" CACHE PATH "Path to a BOOST installation" )

MESSAGE( STATUS "Looking for BOOST: include directory" )
FIND_PATH(
	BOOST_INCLUDE_DIR
	NAMES
		vector_property_map.hpp
		boost/vector_property_map.hpp
	PATHS
		${BOOST_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES
		include
		INCLUDE
		boost_1_34_0/include
		boost_1_33_1/include
		boost_1_33_0/include
)
MESSAGE( STATUS "Looking for BOOST: include directory (${BOOST_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for BOOST: library directory" )
FIND_PATH(
	BOOST_LIBRARY_DIR
	NAMES
		boost_regex
		libboost_regex.a
		libboost_regex.so
		libboost_regex-mt.so
		boost_regex.dll
	PATHS
		${BOOST_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES
		lib
		LIB
		boost_1_34_0/lib
		boost_1_33_1/lib
		boost_1_33_0/lib
)
MESSAGE( STATUS "Looking for BOOST: library directory (${BOOST_LIBRARY_DIR})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( BOOST DEFAULT_MSG BOOST_INCLUDE_DIR BOOST_LIBRARY_DIR )

IF( BOOST_FOUND )
	SET( BOOST_INCLUDE_DIRS "${BOOST_INCLUDE_DIR}" )
	SET( BOOST_LIBRARY_DIRS "${BOOST_LIBRARY_DIR}" )

	# Function making BOOST ready to be used.
	FUNCTION( USE_BOOST )
		IF( NOT BOOST_USED )
			INCLUDE_DIRECTORIES( ${BOOST_INCLUDE_DIRS} )
			SET( BOOST_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using BOOST in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_BOOST )

ENDIF()

IF( BOOST_FOUND OR BOOST_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		BOOST_DIR
		BOOST_INCLUDE_DIR
		BOOST_LIBRARY_DIR
	)
ENDIF()
