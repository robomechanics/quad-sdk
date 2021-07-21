# File:   Build.cmake
# Author: Christian Hoffmann
# Date:   2007--2008
#
# This file is part of the MUSCOD/VPLAN suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2007--2008. All rights reserved.
#
####################################################################################################
#
# Defines macros, variables and user options related to compiling and linking
#
####################################################################################################

IF( NOT _BUILD_ )
SET( _BUILD_ TRUE )

INCLUDE( BuildPaths )
INCLUDE( ProblemHandling )

####################################################################################################

OPTION( BUILD_SHARED_LIBS "ON(default): build shared (.so/.dll) libraries; OFF: build static libraries (.a/.lib)" ON )
MARK_AS_ADVANCED( BUILD_SHARED_LIBS )

####################################################################################################

SET( DEFAULT_RPATH_PROPERTIES
	INSTALL_RPATH_USE_LINK_PATH ON       # use all known link dirs as rpaths
	BUILD_WITH_INSTALL_RPATH OFF         # to produce working executables without "make install"
	INSTALL_RPATH ${INSTALL_LIBRARY_DIR} # find also libs from this package
)

####################################################################################################

# ADD_OPTIONAL_SUBDIRECTORY( <DIR> )
#
# Add <DIR> with ADD_SUBDIRECTORY() if it is present, else ignore it.
MACRO( ADD_OPTIONAL_SUBDIRECTORY AOS_DIR )
	IF( IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${AOS_DIR}" )
		ADD_SUBDIRECTORY( "${CMAKE_CURRENT_SOURCE_DIR}/${AOS_DIR}" )
	ENDIF()
ENDMACRO( ADD_OPTIONAL_SUBDIRECTORY )


# ADD_PACKAGE_LIBRARY( <PACKAGE> <LIBRARY> <SOURCES> <DEPENDENCIES>)
#
# Convenience macro performing several tasks required when a package provides a library:
MACRO( ADD_PACKAGE_LIBRARY _PACKAGE _LIB _SOURCES _DEPS )
	STRING( TOUPPER "${_LIB}" LIB_UC )
	STRING( TOLOWER "${_LIB}" LIB_LC )
	STRING( TOUPPER "${_PACKAGE}" PACKAGE_UC )
	STRING( TOLOWER "${_PACKAGE}" PACKAGE_LC )

	# create abbreviation for convenience
	SET( LIB_TARGET "${PACKAGE_LC}_${LIB_LC}" )

	# define global variable such that the lib can be referred to by other parts of the package
	SET( "${PACKAGE_UC}_${LIB_UC}_LIBRARY" "${LIB_TARGET}" CACHE INTERNAL "" FORCE )

	# add the library to the list of libraries provided by the package
	SET( TEMP "${_PACKAGE}_LIBRARIES;${LIB_TARGET}" )
	LIST( REMOVE_DUPLICATES TEMP )
	SET( "${_PACKAGE}_LIBRARIES" "${TEMP}")
# 	ADD_IF_NOT_PRESENT( "${_PACKAGE}_LIBRARIES" "${LIB_TARGET}" )
# 	MESSAGE( STATUS  "${_PACKAGE}_LIBRARIES: ${${_PACKAGE}_LIBRARIES}" )
	# create the library with required properties
	ADD_LIBRARY( "${LIB_TARGET}" ${_SOURCES} )
	SET_TARGET_PROPERTIES( "${LIB_TARGET}"
		PROPERTIES ${DEFAULT_RPATH_PROPERTIES}
		LINKER_LANGUAGE CXX
	)
	IF( NOT "${_DEPS}" STREQUAL "" )
		TARGET_LINK_LIBRARIES( "${LIB_TARGET}" ${_DEPS} )
	ENDIF()

	# install library
	INSTALL( TARGETS "${LIB_TARGET}"
		DESTINATION ${INSTALL_LIBRARY_DIR}
		PERMISSIONS
			OWNER_READ OWNER_WRITE OWNER_EXECUTE
			GROUP_READ GROUP_EXECUTE
			WORLD_READ WORLD_EXECUTE
		)
ENDMACRO( ADD_PACKAGE_LIBRARY )


# ADD_PACKAGE_EXECUTABLE( <PACKAGE> <EXECUTABLE> <SOURCES> <DEPENDENCIES> )
#
# Convenience macro performing several tasks required when a package provides an executable.
MACRO( ADD_PACKAGE_EXECUTABLE _PACKAGE _EXE _SOURCES _DEPS )
	STRING( TOUPPER "${_EXE}" EXE_UC )
	STRING( TOLOWER "${_EXE}" EXE_LC )
	STRING( TOUPPER "${_PACKAGE}" PACKAGE_UC )
	STRING( TOLOWER "${_PACKAGE}" PACKAGE_LC )

	# define global variable such that the lib can be referred to by other parts of the package
	SET( "${PACKAGE_UC}_${EXE_UC}_EXECUTABLE"
		"${_EXE}" CACHE INTERNAL "" FORCE )

	# create the executable with required properties
	ADD_EXECUTABLE( "${_EXE}" ${_SOURCES} )
	SET_TARGET_PROPERTIES( "${_EXE}"
		PROPERTIES ${DEFAULT_RPATH_PROPERTIES}
		LINKER_LANGUAGE CXX
	)
	IF( NOT "${_DEPS}" STREQUAL "" )
		TARGET_LINK_LIBRARIES( "${_EXE}" ${_DEPS} )
	ENDIF()

	# install EXECUTABLE
	INSTALL( TARGETS "${_EXE}"
		DESTINATION ${INSTALL_EXECUTABLE_DIR}
		PERMISSIONS
			OWNER_READ OWNER_WRITE OWNER_EXECUTE
			GROUP_READ GROUP_EXECUTE
			WORLD_READ WORLD_EXECUTE
	)
ENDMACRO( ADD_PACKAGE_EXECUTABLE )


# ADD_PACKAGE_TESTS( <PACKAGE> <TESTS> <DEPENDENCIES> )
#
# Convenience macro performing several tasks required when a package provides an executable.
MACRO( ADD_PACKAGE_TESTS _PACKAGE _TESTS _DEPS )
	SET( _DOIT ${_PACKAGE}_PROVIDE_SELF_TESTS )
	IF( ${_DOIT} )
		FOREACH( _TEST ${_TESTS} )
			ADD_EXECUTABLE( ${_TEST} ${_TEST} )
			SET_TARGET_PROPERTIES( ${_TEST} PROPERTIES ${DEFAULT_RPATH_PROPERTIES} )
			TARGET_LINK_LIBRARIES( ${_TEST} ${_DEPS} )
			INSTALL( TARGETS ${_TEST}
				DESTINATION ${INSTALL_TEST_DIR}
				PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
			)
	#		ADD_TEST( ${TEST} ${EXECUTABLE_OUTPUT_PATH}/${TEST} )
		ENDFOREACH( _TEST )
	ENDIF()
ENDMACRO( ADD_PACKAGE_TESTS )


ENDIF()