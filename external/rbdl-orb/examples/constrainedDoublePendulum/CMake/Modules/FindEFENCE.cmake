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
# Find an EFENCE installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: EFENCE_FOUND        - TRUE, if the package has been completely found
#   Variable: EFENCE_LIBRARIES    - List of full paths to libraries required for using EFENCE
#   Function: USE_EFENCE()        - Convenience function for preparing CMake for usage of EFENCE.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for EFENCE: " )

SET( EFENCE_DIR "" CACHE PATH "Path to an EFENCE installation" )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################

MESSAGE( STATUS "Looking for EFENCE: library" )
FIND_LIBRARY( EFENCE_LIBRARY
	NAMES efence
	PATHS
		${EFENCE_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for EFENCE: library (${EFENCE_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( EFENCE DEFAULT_MSG EFENCE_LIBRARY )

IF( EFENCE_FOUND )
	SET( EFENCE_LIBRARIES "${EFENCE_LIBRARY}" )

	# Function making EFENCE ready to be used.
	FUNCTION( USE_EFENCE )
		IF( NOT EFENCE_USED )
			# nothing to do
			SET( EFENCE_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using EFENCE in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_EFENCE )

ENDIF()

IF( EFENCE_FOUND OR EFENCE_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		EFENCE_DIR
		EFENCE_LIBRARY
	)
ENDIF()
