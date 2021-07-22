# File:   FindSUNDIALS.cmake
# Author: Jan Albersmeyer
# Author: Christian Hoffmann
# Date:   2009
#
# This file is part of proprietary software of the
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Find an SUNDIALS installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: SUNDIALS_FOUND        - TRUE, if the package has been completely found
#   Variable: SUNDIALS_INCLUDE_DIRS - List of full paths to include directories required for using SUNDIALS
#   Variable: SUNDIALS_LIBRARIES    - List of full paths to libraries requiref for using SUNDIALS
#   Function: USE_SUNDIALS()      - Convenience function for preparing CMake for usage of SUNDIALS.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for SUNDIALS: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( SUNDIALS_DIR "" CACHE PATH "Path to an ADOL-C installation" )

MESSAGE( STATUS "Looking for SUNDIALS: include directory" )
FIND_PATH( SUNDIALS_INCLUDE_DIR sundials_types.h
	PATHS
		${SUNDIALS_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE include/sundials
)

MESSAGE( STATUS "Looking for SUNDIALS: include directory (${SUNDIALS_INCLUDE_DIR})" )

#MESSAGE( STATUS "Looking for SUNDIALS: cvode library" )
#FIND_LIBRARY( SUNDIALS_CVODE_LIBRARY
#	NAMES sundials_cvode
#	PATHS
#		${SUNDIALS_DIR}
#		${DEFAULT_PACKAGE_DIRS}
#		${DEFAULT_LIBRARY_DIRS}
#	PATH_SUFFIXES lib LIB
#)
#MESSAGE( STATUS "Looking for SUNDIALS: cvode library (${SUNDIALS_CVODE_LIBRARY})" )

MESSAGE( STATUS "Looking for SUNDIALS: cvodes library" )
FIND_LIBRARY( SUNDIALS_CVODES_LIBRARY
	NAMES sundials_cvodes
	PATHS
		${SUNDIALS_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for SUNDIALS: cvodes library (${SUNDIALS_CVODES_LIBRARY})" )

#MESSAGE( STATUS "Looking for SUNDIALS: ida library" )
#FIND_LIBRARY( SUNDIALS_IDA_LIBRARY
#	NAMES sundials_ida
#	PATHS
#		${SUNDIALS_DIR}
#		${DEFAULT_PACKAGE_DIRS}
#		${DEFAULT_LIBRARY_DIRS}
#	PATH_SUFFIXES lib LIB
#)
#MESSAGE( STATUS "Looking for SUNDIALS: ida library (${SUNDIALS_IDA_LIBRARY})" )

MESSAGE( STATUS "Looking for SUNDIALS: idas library" )
FIND_LIBRARY( SUNDIALS_IDAS_LIBRARY
	NAMES sundials_idas
	PATHS
		${SUNDIALS_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for SUNDIALS: idas library (${SUNDIALS_IDAS_LIBRARY})" )

MESSAGE( STATUS "Looking for SUNDIALS: kinsol library" )
FIND_LIBRARY( SUNDIALS_KINSOL_LIBRARY
	NAMES sundials_kinsol
	PATHS
		${SUNDIALS_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib LIB
)
MESSAGE( STATUS "Looking for SUNDIALS: kinsol library (${SUNDIALS_KINSOL_LIBRARY})" )

MESSAGE( STATUS "Looking for SUNDIALS: nvecserial library" )
FIND_LIBRARY( SUNDIALS_NVECSERIAL_LIBRARY
	NAMES sundials_nvecserial
	PATHS
		${SUNDIALS_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for SUNDIALS: nvecserial library (${SUNDIALS_NVECSERIAL_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
	SUNDIALS
	DEFAULT_MSG
#	SUNDIALS_CVODE_LIBRARY
	SUNDIALS_CVODES_LIBRARY
#	SUNDIALS_IDA_LIBRARY
	SUNDIALS_IDAS_LIBRARY
	SUNDIALS_KINSOL_LIBRARY
	SUNDIALS_NVECSERIAL_LIBRARY
	SUNDIALS_INCLUDE_DIR
	)

IF( SUNDIALS_FOUND )
	SET( INCLUDE_BASE "${SUNDIALS_INCLUDE_DIR}/../")

	SET( SUNDIALS_INCLUDE_DIRS
		"${SUNDIALS_INCLUDE_DIR}"
		"${INCLUDE_BASE}"
#		"${INCLUDE_BASE}/cvode"
		"${INCLUDE_BASE}/cvodes"
#		"${INCLUDE_BASE}/ida"
		"${INCLUDE_BASE}/idas"
		"${INCLUDE_BASE}/kinsol"
		"${INCLUDE_BASE}/nvector"
		"${INCLUDE_BASE}/sundials"
		)

	SET( SUNDIALS_LIBRARIES
#		"${SUNDIALS_CVODE_LIBRARY}"
		"${SUNDIALS_CVODES_LIBRARY}"
#		"${SUNDIALS_IDA_LIBRARY}"
		"${SUNDIALS_IDAS_LIBRARY}"
		"${SUNDIALS_KINSOL_LIBRARY}"
		"${SUNDIALS_NVECSERIAL_LIBRARY}"
	)

	# Function making SUNDIALS ready to be used.
	FUNCTION( USE_SUNDIALS )
		IF( NOT SUNDIALS_USED )
			INCLUDE_DIRECTORIES( ${SUNDIALS_INCLUDE_DIRS} )
			SET( SUNDIALS_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using SUNDIALS in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_SUNDIALS )

ENDIF()

IF( SUNDIALS_FOUND OR SUNDIALS_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		SUNDIALS_DIR
		SUNDIALS_INCLUDE_DIR
#		SUNDIALS_CVODE_LIBRARY
		SUNDIALS_CVODES_LIBRARY
#		SUNDIALS_IDA_LIBRARY
		SUNDIALS_IDAS_LIBRARY
		SUNDIALS_KINSOL_LIBRARY
		SUNDIALS_NVECSERIAL_LIBRARY
		)
ENDIF()
