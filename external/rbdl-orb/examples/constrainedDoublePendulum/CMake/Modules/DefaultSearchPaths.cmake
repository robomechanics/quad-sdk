# File:   DefaultSearchPaths.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Adds variables with search paths for packages, libraries and includes.
#
# Defines:
#   - SUITE_DIR              Top-level dir of a MUSCOD/VPLAN suite, cached.
#                            NOTE: takes value from environment variable "SUITE_DIR" (if set)
#   - PACKAGES_DIR           Dir containing packages, cached.
#                            NOTE: takes value from  environment variable "PACKAGES_DIR" (if set)
#   - DEFAULT_PACKAGE_DIRS   List of dirs of all suitable packages found in PACKAGES_DIR, uncached.
#                            NOTE: only packages with the same CMAKE_BUILD_TYPE as the current
#                            project are found!
#   - DEFAULT_INCLUDE_DIRS   List of system include dirs, platform specific, uncached.
#   - DEFAULT_LIBRARY_DIRS   List of system library dirs, platform specific, uncached.
#
####################################################################################################

IF( NOT _DEFAULTSEARCHPATHS_ )
SET( _DEFAULTSEARCHPATHS_ TRUE )



####################################################################################################
#### DETERMINE PATHS TO SUITES / PACKAGES
####################################################################################################
# Paths are tried to be read from environment variables. However, they are overridden by
# user-specified values (from the ccmake gui or or by calling cmake with the -D option) if specified.
# All relative paths are converted to absolute representations.

IF( SUITE_DIR )
	GET_FILENAME_COMPONENT( ABS_PATH "${SUITE_DIR}/" ABSOLUTE )
	SET( SUITE_DIR "${ABS_PATH}" CACHE PATH "Path of the suite root dir." FORCE )
	MESSAGE( STATUS  "Using user-specified path to suite: '${SUITE_DIR}'")
ELSE( SUITE_DIR )
	IF( IS_DIRECTORY "$ENV{SUITE_DIR}" )
		GET_FILENAME_COMPONENT( ABS_PATH "$ENV{SUITE_DIR}" ABSOLUTE )
		SET( SUITE_DIR "${ABS_PATH}" CACHE PATH "Path of the suite root dir." FORCE )
		MESSAGE( STATUS  "Using path of suite as defined in environment variable 'SUITE_DIR': '${SUITE_DIR}'" )
	ELSE( IS_DIRECTORY "$ENV{SUITE_DIR}" )
		MESSAGE( STATUS  "No valid environment variable 'SUITE_DIR' found, using built-in search paths." )
		SET( SUITE_DIR "" CACHE PATH "Path of the suite root dir." FORCE )
	ENDIF()
ENDIF()

IF( PACKAGES_DIR )
	GET_FILENAME_COMPONENT( ABS_PATH "${PACKAGES_DIR}" ABSOLUTE )
	SET( PACKAGES_DIR "${ABS_PATH}" CACHE PATH "Path to a packages dir." FORCE )
	MESSAGE( STATUS  "Using user-specified path to the packages dir: '${PACKAGES_DIR}'")
ELSE( PACKAGES_DIR )
	IF( IS_DIRECTORY "$ENV{PACKAGES_DIR}" )
		GET_FILENAME_COMPONENT( ABS_PATH "$ENV{PACKAGES_DIR}" ABSOLUTE )
		SET( PACKAGES_DIR "${ABS_PATH}" CACHE PATH "Path to a packages dir." FORCE )
		MESSAGE( STATUS  "Using path to packages dir defined in environment variable : '${PACKAGES_DIR}'" )
	ELSE( IS_DIRECTORY "$ENV{PACKAGES_DIR}" )
		MESSAGE( STATUS  "No valid environment variable 'PACKAGES_DIR' found. using built-in search paths." )
		SET( PACKAGES_DIR "${PACKAGES_DIR}" CACHE PATH "Path to a packages dir." )
	ENDIF()
ENDIF()

MARK_AS_ADVANCED(
	SUITE_DIR
	PACKAGES_DIR
)


####################################################################################################
#### DEFAULT (SYSTEM) SEARCH PATHS FOR INCLUDES/LIBRARIES
####################################################################################################
# Used as defaults for INCLUDE_DIRECTORIES() and FIND_LIBRARY() commands
SET( INCLUDE_SEARCH_DIRS
	# win32
	c:/Programme/MinGW/include
	c:/Programme/Develop/MinGW/include
	# unix
	/include
	/usr/include
	/usr/local/include
	/usr/local/irun/include
	/opt # sebastian walters boost installation
)

SET( LIBRARY_SEARCH_DIRS
	# win32
	c:/Programme/MinGW/lib
	c:/Programme/Develop/MinGW/lib
	c:/Programme/cygwin/lib/gcc/i686-pc-cygwin/3.4.
	c:/Programme/cygwin/lib/gcc/i686-pc-mingw32/3.4.4
	d:/Programs/MinGW/lib
	d:/MATH/MinGW/lib
	# unix
	/lib64
	/usr/lib64
	/usr/local/lib64
	/usr/local/irun/lib64
	/lib
	/usr/lib
	/usr/local/lib
	/usr/local/irun/lib
	/opt
)

SET( PACKAGE_SEARCH_DIRS
	# platform-independent paths
# 	.
	${PACKAGES_DIR}
	${SUITE_DIR}
	${SUITE_DIR}/Packages
	${SUITE_DIR}/PACKAGES
	${SUITE_DIR}/packages
	# win32
	# unix
	/lib64 # some packages are installed to lib dirs...
	/usr/lib64
	/usr/local/lib64
	/usr/share # ... and some to share dirs
	/usr/local/share
	/lib # try 32-bit dirs latest to ensure 64-bit versions are always found first
	/usr/lib
	/usr/local/lib
	/opt
)


####################################################################################################
#### BUILD LIST OF PACKAGE SEARCH PATHS
####################################################################################################
FOREACH( IDIR ${INCLUDE_SEARCH_DIRS} )
	IF( EXISTS "${IDIR}" )
		LIST( APPEND DEFAULT_INCLUDE_DIRS "${IDIR}" )
	ENDIF()
ENDFOREACH( IDIR )

FOREACH( LDIR ${LIBRARY_SEARCH_DIRS} )
	IF( EXISTS "${LDIR}" )
		LIST( APPEND DEFAULT_LIBRARY_DIRS "${LDIR}" )
	ENDIF()
ENDFOREACH( LDIR )

# Determine package search paths to be used by FindXXX.cmake scripts and FIND_XXX() commands
# Packages are searched non-recursively (in the following order) in the:
# - current dir
# - suite dir and packages subfolder(s) (if specified)
# - packages dir (if specified)
# NOTE: The _first_ found package in these directories will be used.

# Adds each subfolder "XXX" of the previously defined package search dirs to the default packages
# list. If XXX has a sub-folder with a build type fitting to the CMAKE_BUILD_TYPE of the current
# that folder will be added instead of XXX.
#
# FIXME: the "ABSOLUTE" doesn't seem to recognize symbolic links, which leads to double listings
# of the same packages when using a linked packages dir
FOREACH( PACKAGES_SEARCH_DIR ${PACKAGE_SEARCH_DIRS} )
	GET_FILENAME_COMPONENT( PACKAGES_SEARCH_DIR "${PACKAGES_SEARCH_DIR}" ABSOLUTE )
	IF( IS_DIRECTORY "${PACKAGES_SEARCH_DIR}" )
		MESSAGE( STATUS  "Looking for packages in '${PACKAGES_SEARCH_DIR}'" )
		FILE( GLOB SEARCH_DIR_CONTENTS ${PACKAGES_SEARCH_DIR}/*/ )
		FOREACH( SEARCH_DIR_CONTENT ${SEARCH_DIR_CONTENTS} )
			IF( IS_DIRECTORY ${SEARCH_DIR_CONTENT} )
				IF( EXISTS ${SEARCH_DIR_CONTENT}/${CMAKE_BUILD_TYPE} )
					SET( PACKAGE_DIR ${SEARCH_DIR_CONTENT}/${CMAKE_BUILD_TYPE} )
				ELSE( EXISTS ${SEARCH_DIR_CONTENT}/${CMAKE_BUILD_TYPE} )
					SET( PACKAGE_DIR ${SEARCH_DIR_CONTENT} )
				ENDIF()
				GET_FILENAME_COMPONENT( PACKAGE_DIR "${PACKAGE_DIR}" ABSOLUTE )
				LIST( APPEND DEFAULT_PACKAGE_DIRS "${PACKAGE_DIR}" )
# 				MESSAGE( STATUS  "  Assuming a package in '${PACKAGE_DIR}'" )
			ENDIF()
		ENDFOREACH( SEARCH_DIR_CONTENT )
	ENDIF()
ENDFOREACH( PACKAGES_SEARCH_DIR )

ENDIF()
