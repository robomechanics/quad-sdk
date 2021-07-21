# File:   FindADIFOR2.cmake
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
# Find an installation of ADIFOR2
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: ADIFOR2_FOUND      - TRUE is the package has been completely found
#   Variable: ADIFOR2_DIR        - Directory where ADIFOR2 was found (on success) [cached]
#   Variable: ADIFOR2_EXECUTABLE - the ADIFOR2 main executable (on success) [cached]
#   Function: USE_ADIFOR2()      - Convenience function for preparing CMake for usage of ADIFOR2.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for ADIFOR2: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
MESSAGE( STATUS "Looking for ADIFOR2: installation root dir" )
FIND_PATH( ADIFOR2_DIR
	NAMES FINDME-ADIFOR2.txt
	PATHS ${DEFAULT_PACKAGE_DIRS}
	DOC "Root directory of an ADIFOR2 installation."
	PATH_SUFFIXES Src
)
MESSAGE( STATUS "Looking for ADIFOR2: installation root dir (${ADIFOR2_DIR})" )

MESSAGE( STATUS "Looking for ADIFOR2: executable" )
FIND_PROGRAM( ADIFOR2_EXECUTABLE
	NAMES Adifor2.1-Linux
	PATHS
		${ADIFOR2_DIR}
		${DEFAULT_PACKAGE_DIRS}
	PATH_SUFFIXES
		ADIFOR2.0/bin
		adifor2.0/bin
	DOC "The ADIFOR2 main executable."
)
MESSAGE( STATUS "Looking for ADIFOR2: executable (${ADIFOR2_EXECUTABLE})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	ADIFOR2
	"Could not find ADIFOR2. Note: currently only linux versions are supported."
	ADIFOR2_DIR ADIFOR2_EXECUTABLE
)

IF( ADIFOR2_FOUND )
	FUNCTION( USE_ADIFOR2 )
		IF( NOT ADIFOR2_USED )
			# nothing to do
			SET( ADIFOR2_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using ADIFOR2 in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_ADIFOR2 )
ENDIF()

IF( ADIFOR2_FOUND OR ADIFOR2_FIND_QUITELY )
	MARK_AS_ADVANCED(
		ADIFOR2_DIR
		ADIFOR2_EXECUTABLE
	)
ENDIF()
