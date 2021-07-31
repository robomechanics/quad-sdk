# File:   FindQPOASES.cmake
# Author: Jan Albersmeyer
# Date:   2009
#
#
####################################################################################################
#
# Find an QPOASES installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines
#   Variable: QPOASES_FOUND        - TRUE, if the package has been completely found
#   Variable: QPOASES_INCLUDE_DIRS - List of full paths to include directories required for using QPOASES
#   Variable: QPOASES_LIBRARIES    - List of full paths to libraries required for using QPOASES
#   Variable: QPOASES_OCT_LINK_LIBRARIES    - List of libraries required linking an oct file [cached]
#   Function: USE_QPOASES()      - Convenience function for preparing CMake for usage of QPOASES.
#
# TODO: hard-coded linker string in QPOASES_OCT_LINK_LIBRARIES
# TODO: hard-coded gfortran-dependency in QPOASES_OCT_LINK_LIBRARIES
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for QPOASES: " )


####################################################################################################
#### SEARCH PACKAGE COMPONENTS
####################################################################################################
SET( QPOASES_DIR "" CACHE PATH "Path to an QPOASES installation" )

MESSAGE( STATUS "Looking for QPOASES: include directory" )
FIND_PATH( QPOASES_INCLUDE_DIR QProblem.hpp
	PATHS
		${QPOASES_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_INCLUDE_DIRS}
	PATH_SUFFIXES include INCLUDE
)
MESSAGE( STATUS "Looking for QPOASES: include directory (${QPOASES_INCLUDE_DIR})" )

MESSAGE( STATUS "Looking for QPOASES: base library" )
FIND_LIBRARY( QPOASES_BASE_LIBRARY
	NAMES qpOASES
	PATHS
		${QPOASES_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB src SRC
)
MESSAGE( STATUS "Looking for QPOASES: base library (${QPOASES_BASE_LIBRARY})" )

MESSAGE( STATUS "Looking for QPOASES: extras library" )
FIND_LIBRARY( QPOASES_EXTRAS_LIBRARY
	NAMES qpOASESExtras
	PATHS
		${QPOASES_DIR}
		${DEFAULT_PACKAGE_DIRS}
		${DEFAULT_LIBRARY_DIRS}
	PATH_SUFFIXES lib  lib64 LIB src SRC
)
MESSAGE( STATUS "Looking for QPOASES: extras library (${QPOASES_EXTRAS_LIBRARY})" )


####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( QPOASES DEFAULT_MSG
	QPOASES_BASE_LIBRARY
	QPOASES_EXTRAS_LIBRARY
	QPOASES_INCLUDE_DIR
)

IF( QPOASES_FOUND )
	SET( QPOASES_INCLUDE_DIRS 
		"${QPOASES_INCLUDE_DIR}"
		"${QPOASES_INCLUDE_DIR}/EXTRAS/"
		"${QPOASES_INCLUDE_DIR}/../SRC/"
		"${QPOASES_INCLUDE_DIR}/../SRC/EXTRAS/"
	)

	SET (QPOASES_LIBRARIES
		"${QPOASES_INTERP_LIBRARY}"
		"${QPOASES_BASE_LIBRARY}"
		"${QPOASES_EXTRAS_LIBRARY}"
	)

	# Function making QPOASES ready to be used.
	FUNCTION( USE_QPOASES )
		IF( NOT QPOASES_USED )
			INCLUDE_DIRECTORIES( ${QPOASES_INCLUDE_DIRS} )
			SET( QPOASES_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using QPOASES in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_QPOASES )

ENDIF( QPOASES_FOUND )

IF( QPOASES_FOUND OR QPOASES_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		QPOASES_DIR
		QPOASES_INCLUDE_DIR
		QPOASES_BASE_LIBRARY
		QPOASES_EXTRAS_LIBRARY
	)
ENDIF( QPOASES_FOUND OR QPOASES_FIND_QUIETLY )
