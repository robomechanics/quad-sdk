# File:   FindJMODELICA.cmake
# Author: Sebastian Sager
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of proprietary software of the
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
###################################################################################################
#
# Find the JMODELICA includes and libraries
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: JMODELICA_FOUND        - TRUE, if the package has been completely found
#   Variable: JMODELICA_INCLUDE_DIRS - List of full paths to include directories required for using JMODELICA
#   Variable: JMODELICA_LIBRARIES    - List of full paths to libraries required for using JMODELICA
#   Function: USE_JMODELICA()      - Convenience function for preparing CMake for usage of JMODELICA.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for JMODELICA: " )


####################################################################################################
#### SEARCH REQUIRED PACKAGE RESOURCES
####################################################################################################
#SET( JMODELICA_DIR "" CACHE PATH "/home/ssager/solver/JModelica/build/include" )
SET( JMODELICA_DIR "/home/ssager/solver/JModelica/build/" )
SET( JMI_LIBNAME "libjmi.a" )
SET( JMI_ALG_LIBNAME "libjmi_algorithm.a" )
SET( JMI_SOL_LIBNAME "libjmi_solver.a" )

MESSAGE( STATUS "Looking for JMODELICA: include directory" )
FIND_PATH( JMODELICA_INCLUDE_DIR "jmi.h"
	PATHS
		${JMODELICA_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES JMODELICA/Include JMODELICA/INCLUDE JMODELICA/include Include INCLUDE include
)
MESSAGE( STATUS "Looking for JMODELICA: include directory (${JMODELICA_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for JMODELICA: library" )
FIND_LIBRARY( JMI_LIBRARY
	NAMES ${JMI_LIBNAME}
	PATHS
		${JMODELICA_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES JMODELICA/Lib JMODELICA/lib JMODELICA/LIB Lib lib  lib64 LIB
)
MESSAGE( STATUS "Looking for JMODELICA: library (${JMI_LIBRARY})" )
MESSAGE( STATUS "Looking for JMODELICA: library" )
FIND_LIBRARY( JMI_ALG_LIBRARY
	NAMES ${JMI_ALG_LIBNAME}
	PATHS
		${JMODELICA_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES JMODELICA/Lib JMODELICA/lib JMODELICA/LIB Lib lib  lib64 LIB
)
MESSAGE( STATUS "Looking for JMODELICA: library (${JMI_LIBRARY})" )
MESSAGE( STATUS "Looking for JMODELICA: library" )
FIND_LIBRARY( JMI_SOL_LIBRARY
	NAMES ${JMI_SOL_LIBNAME}
	PATHS
		${JMODELICA_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES JMODELICA/Lib JMODELICA/lib JMODELICA/LIB Lib lib  lib64 LIB
)
MESSAGE( STATUS "Looking for JMODELICA: library (${JMI_LIBRARY})" )

####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( JMODELICA DEFAULT_MSG
	JMI_LIBRARY
	JMI_ALG_LIBRARY
	JMI_SOL_LIBRARY
	JMODELICA_INCLUDE_DIR
)

IF( JMODELICA_FOUND )
	SET( JMODELICA_INCLUDE_DIRS
		"${JMODELICA_INCLUDE_DIR}"
	)
	SET( JMODELICA_LIBRARIES
		"${JMI_LIBRARY}"
		"${JMI_ALG_LIBRARY}"
		"${JMI_SOL_LIBRARY}"
	)

	# Function making JMODELICA ready to be used.
	FUNCTION( USE_JMODELICA )
		IF( NOT JMODELICA_USED )
			INCLUDE_DIRECTORIES( ${JMODELICA_INCLUDE_DIRS} )
			SET( JMODELICA_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using JMODELICA in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_JMODELICA )
ENDIF()

IF( JMODELICA_FOUND OR JMODELICA_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		JMODELICA_DIR
		JMODELICA_INCLUDE_DIR
		JMI_LIBRARY
		JMI_ALG_LIBRARY
		JMI_SOL_LIBRARY
	)
ENDIF()
