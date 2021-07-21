# File:   FindMATLAB.cmake
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
# Find a MATLAB installation
#
# This file fulfils the CMAKE modules guidelines:
#   http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   Variable: MATLAB_FOUND        - TRUE, if the package has been completely found
#   Variable: MATLAB_INCLUDE_DIRS - List of full paths to include directories required for using MATLAB
#   Variable: MATLAB_LIBRARIES    - List of full paths to libraries required for using MATLAB
#   Function: USE_MATLAB()      - Convenience function for preparing CMake for usage of MATLAB.
#
###################################################################################################

INCLUDE( DefaultSearchPaths )
INCLUDE( FindPackageHandleStandardArgs )

MESSAGE( STATUS "Looking for MATLAB: " )

SET( MATLAB_DIR "" CACHE PATH "Path to an MATLAB installation" )


####################################################################################################
#### PLATFORM DEPENDENT SETTINGS
####################################################################################################
IF( WIN32 )

	SET( MEX_NAME "libmex" )
	SET( MX_NAME  "libmx" )
	SET( ENG_NAME "libeng" )
	SET( MAT_NAME "libmat" )

	SET( MATLAB_INCLUDE_SEARCH_DIRS
		${MATLAB_DIR}
		${DEFAULT_PACKAGE_DIRS}
		"C:/Programme/Matlab71/extern/include"
		${DEFAULT_INCLUDE_DIRS}
	)

	SET( MATLAB_LIBRARY_SEARCH_DIRS
		${MATLAB_DIR}
		${DEFAULT_PACKAGE_DIRS}
		"C:/Programme/Matlab71/bin/win32"
		${DEFAULT_LIBRARY_DIRS}
	)

ELSE( WIN32 )

	SET( MEX_NAME "mex" )
	SET( MX_NAME  "mx" )
	SET( ENG_NAME "eng" )
	SET( MAT_NAME "mat" )

	SET( MATLAB_INCLUDE_SEARCH_DIRS
		${MATLAB_DIR}/extern/include/
		${DEFAULT_PACKAGE_DIRS}
		"/usr/local/matlab/extern/include/"
		"/usr/local/Matlab/extern/include/"
		"/usr/local/Matlab-7sp1/extern/include/"
		"/opt/Matlab-7sp1/extern/include/"
		"$ENV{HOME}/Matlab-7sp1/extern/include/"
		"$ENV{HOME}/redhat-Matlab/extern/include/"
		${DEFAULT_INCLUDE_DIRS}
	)
	IF( CMAKE_SIZEOF_VOID_P EQUAL 4 ) # Regular x86
		SET( MATLAB_LIBRARY_SEARCH_DIRS
			${MATLAB_DIR}/bin/glnx86
			${DEFAULT_PACKAGE_DIRS}
			/usr/local/matlab/bin/glnx86/
			/usr/local/Matlab/bin/glnx86/
			/usr/local/Matlab-7sp1/bin/glnx86/
			/opt/Matlab-7sp1/bin/glnx86/
			$ENV{HOME}/Matlab-7sp1/bin/glnx86/
			$ENV{HOME}/redhat-Matlab/bin/glnx86/
			${DEFAULT_LIBRARY_DIRS}
		)
	ELSE( CMAKE_SIZEOF_VOID_P EQUAL 4 ) # NOT 4 -> 8 -> AMD64:
		SET( MATLAB_LIBRARY_SEARCH_DIRS
			${MATLAB_DIR}/bin/glnxa64
			${DEFAULT_PACKAGE_DIRS}
			/usr/local/matlab/bin/glnxa64/
			/usr/local/Matlab/bin/glnxa64/
			/usr/local/Matlab-7sp1/bin/glnxa64/
			/opt/Matlab-7sp1/bin/glnxa64/
			$ENV{HOME}/Matlab7_64/bin/glnxa64/
			$ENV{HOME}/Matlab-7sp1/bin/glnxa64/
			$ENV{HOME}/redhat-Matlab/bin/glnxa64/
			${DEFAULT_LIBRARY_DIRS}
		)
	ENDIF()

ENDIF()


####################################################################################################
#### SEARCH INCLUDES
####################################################################################################
MESSAGE( STATUS "Looking for MATLAB: include directory" )
FIND_PATH( MATLAB_INCLUDE_DIR mex.h
	PATHS ${MATLAB_INCLUDE_SEARCH_DIRS}
	PATH_SUFFIXES include INCLUDE
)
MESSAGE( STATUS "Looking for MATLAB: include directory" MATLAB_INCLUDE_DIR )


####################################################################################################
#### SEARCH LIBRARIES
####################################################################################################

MESSAGE( STATUS "Looking for MATLAB: mex library" )
FIND_LIBRARY( MATLAB_MEX_LIBRARY "${MEX_NAME}"
	PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for MATLAB: mex library" MATLAB_MEX_LIBRARY )

MESSAGE( STATUS "Looking for MATLAB: mx library" )
FIND_LIBRARY( MATLAB_MX_LIBRARY  "${MX_NAME}"
	PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for MATLAB: mx library" MATLAB_MX_LIBRARY )

MESSAGE( STATUS "Looking for MATLAB: eng library" )
FIND_LIBRARY( MATLAB_ENG_LIBRARY "${ENG_NAME}"
	PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
	PATH_SUFFIXES lib  lib64 LIB
)
MESSAGE( STATUS "Looking for MATLAB: eng library" MATLAB_ENG_LIBRARY )

MESSAGE( STATUS "Looking for MATLAB: mat library" )
FIND_LIBRARY( MATLAB_MAT_LIBRARY "${MAT_NAME}"
	PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
	PATH_SUFFIXES lib  lib64 LIB Lib
)
MESSAGE( STATUS "Looking for MATLAB: mat library" MATLAB_MAT_LIBRARY )


IF (NOT WIN32)

	FIND_LIBRARY( MATLAB_UT_LIBRARY "ut"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_MWSERVICES_LIBRARY "mwservices"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_MWMPATH_LIBRARY "mwmpath"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_MWMDISPATCHER_LIBRARY "mwm_dispatcher"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_ICUDATA_LIBRARY "icudata"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_ICUUC_LIBRARY "icuuc"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_ICUI18N_LIBRARY "icui18n"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)
	FIND_LIBRARY( MATLAB_ICUIO_LIBRARY "icuio"
		PATHS ${MATLAB_LIBRARY_SEARCH_DIRS}
		PATH_SUFFIXES lib  lib64 LIB Lib
	)

	SET (MATLAB_EXTRA_LIBRARIES
		${MATLAB_UT_LIBRARY}
		${MATLAB_MWSERVICES_LIBRARY}
		${MATLAB_MWMPATH_LIBRARY}
		${MATLAB_MWMDISPATCHER_LIBRARY}
		${MATLAB_ICUDATA_LIBRARY}
		${MATLAB_ICUUC_LIBRARY}
		${MATLAB_ICUI18N_LIBRARY}
		${MATLAB_ICUIO_LIBRARY}
	)

ELSE (NOT WIN32)

	SET (MATLAB_EXTRA_LIBRARIES "")

ENDIF()

####################################################################################################
#### EVALUATE SEARCH RESULTS
####################################################################################################
FIND_PACKAGE_HANDLE_STANDARD_ARGS( MATLAB DEFAULT_MSG
	MATLAB_INCLUDE_DIR
	MATLAB_MEX_LIBRARY
	MATLAB_MX_LIBRARY
	MATLAB_ENG_LIBRARY
	MATLAB_MAT_LIBRARY
)

IF( MATLAB_FOUND )
	# check for mwSize, use int as a replacement
	INCLUDE ( CheckTypeSize )
	SET(CMAKE_EXTRA_INCLUDE_FILES ${MATLAB_INCLUDE_DIR}/mat.h)
	CHECK_TYPE_SIZE ( mwSize MATLAB_MWSIZE_SIZE )
	SET(CMAKE_EXTRA_INCLUDE_FILES)

	IF ( MATLAB_MWSIZE_SIZE )
		SET ( MATLAB_MWSIZE_TYPE "mwSize" )
	ELSE ( MATLAB_MWSIZE_SIZE )
		SET ( MATLAB_MWSIZE_TYPE "int" )
	ENDIF()

	SET( MATLAB_INCLUDE_DIRS ${MATLAB_INCLUDE_DIR} )
	SET( MATLAB_LIBRARIES
		"${MATLAB_MEX_LIBRARY}"
		"${MATLAB_MX_LIBRARY}"
		"${MATLAB_ENG_LIBRARY}"
		"${MATLAB_MAT_LIBRARY}"
		"${MATLAB_EXTRA_LIBRARIES}"
	)

	# Function making MATLAB ready to be used.
	FUNCTION( USE_MATLAB )
		IF( NOT MATLAB_USED )
			INCLUDE_DIRECTORIES( ${MATLAB_INCLUDE_DIRS} )
			SET( MATLAB_USED TRUE )
		ENDIF()
		MESSAGE( STATUS "Using MATLAB in ${CMAKE_CURRENT_SOURCE_DIR}" )
	ENDFUNCTION( USE_MATLAB )

ENDIF()

IF( MATLAB_FOUND OR MATLAB_FIND_QUIETLY )
	MARK_AS_ADVANCED(
		MATLAB_DIR
		MATLAB_INCLUDE_DIR
		MATLAB_MAT_LIBRARY
		MATLAB_MEX_LIBRARY
		MATLAB_MX_LIBRARY
		MATLAB_ENG_LIBRARY
		MATLAB_UT_LIBRARY
		MATLAB_MWSERVICES_LIBRARY
		MATLAB_MWMPATH_LIBRARY
		MATLAB_MWMDISPATCHER_LIBRARY
		MATLAB_ICUDATA_LIBRARY
		MATLAB_ICUUC_LIBRARY
		MATLAB_ICUI18N_LIBRARY
		MATLAB_ICUIO_LIBRARY
	)
ENDIF()
