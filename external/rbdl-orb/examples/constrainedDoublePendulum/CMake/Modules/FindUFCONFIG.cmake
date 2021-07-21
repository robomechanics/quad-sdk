# File:   FindUFCONFIG.cmake
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
# Find the UFCONFIG includes and libraries
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: UFCONFIG_FOUND        - TRUE, if the package has been completely found
#   Variable: UFCONFIG_INCLUDE_DIRS - List of full paths to include directories required for using UFCONFIG
#   Function: USE_UFCONFIG()      - Convenience function for preparing CMake for usage of UFCONFIG.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for UFCONFIG: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( UFCONFIG_DIR "" CACHE PATH "Path to an UFCONFIG installation" )

MESSAGE( STATUS "Looking for UFCONFIG: include directory" )
FIND_PATH( UFCONFIG_INCLUDE_DIR UFconfig.h
	PATHS
		${UFCONFIG_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES UFconfig UFCONFIG Include INCLUDE include
)
MESSAGE( STATUS "Looking for UFCONFIG: include directory (${UFCONFIG_INCLUDE_DIR})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( UFCONFIG DEFAULT_MSG UFCONFIG_INCLUDE_DIR )

IF( UFCONFIG_FOUND )
	SET( UFCONFIG_INCLUDE_DIRS "${UFCONFIG_INCLUDE_DIR}" )

	# Function making UFCONFIG ready to be used.
	FUNCTION( USE_UFCONFIG )
		IF( NOT UFCONFIG_USED )
			INCLUDE_DIRECTORIES( ${UFCONFIG_INCLUDE_DIRS} )
			SET( UFCONFIG_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using UFCONFIG in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_UFCONFIG )

ENDIF()

IF( UFCONFIG_FOUND OR UFCONFIG_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		UFCONFIG_DIR
		UFCONFIG_INCLUDE_DIR
	)
ENDIF()
