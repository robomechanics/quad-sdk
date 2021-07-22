# File:   BuildVplanApps.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of the VPLAN Suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2007--2009 Christian Hoffmann, christian.hoffmann@iwr.uni-heidelberg.de
# All rights reserved. For details, see file 'LICENSE'.
#
####################################################################################################
#
# Convenience macros when building applications for VPLAN.
#
####################################################################################################
IF( NOT _BUILDVPLANAPPS_ )
SET( _BUILDVPLANAPPS_ TRUE )


####################################################################################################

INCLUDE( ProblemHandling )
INCLUDE( BuildFortran )
INCLUDE( CompilerSettingsFortran )

####################################################################################################

# ADD_VPLAN_APP( <LIBRARY> <INIFILE> [<PPFLAGS>] )
#
# Create a problem library for VPLAN application.
#
# Preconditions:
# * <LIBRARY> is not empty
# * <INIFILE> exists and is readable
# * The VPLAN package has been found ( by FIND_PACKAGE(VPLAN) ) and used (by USE_VPLAN).
#
# Calls VPLAN's 'doit' to generate derivatives required by the VPLAN problem description file
# <INIFILE>. From the generated sources the library target <LIBRARY> is created, passing <PPFLAGS>
# as flags to the preprocessor.
#
# The library name <LIBRARY> is used to create a meaningful subdirectory for intermediate files
# created during derivative generation. They are deleted when the global 'clean' target it built.
FUNCTION( ADD_VPLAN_APP _LIB _INI )
	MESSAGE( STATUS  "Building VPLAN problem library '${_LIB}'." )

	# Check preconditions
	SET( _VALID_ARGS TRUE )
	IF( "${_LIB}" STREQUAL "" )
		ERROR( "ADD_VPLAN_APP(): name of target problem library is empty." )
		SET( _VALID_ARGS FALSE )
	ENDIF()
	IF( NOT EXISTS "${_INI}" )
		ERROR( "ADD_VPLAN_APP(): cannot read problem ini file '${_INI}'." )
		SET( _VALID_ARGS FALSE )
	ENDIF()
	IF( NOT VPLAN_FOUND )
		ERROR( "ADD_VPLAN_APP(): Required package VPLAN not found." )
		SET( _VALID_ARGS FALSE )
	ENDIF()

	# Parse optional arguments
	IF( ARGC GREATER 2 )
		SET( _PPFLAGS "${ARGV2}" )
	ELSE()
		SET( _PPFLAGS )
	ENDIF()

	# Create problem library
	IF( CMAKE_BUILD_TYPE AND _VALID_ARGS )

		# Create working dir for generated files
		SET( _WORKDIR "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${_LIB}_doit_temp" )
		FILE( MAKE_DIRECTORY "${_WORKDIR}" )

		# Call doit to generate derivatives
		MESSAGE( STATUS  "Calling doit. Command: '${VPLAN_DOIT_EXECUTABLE} ${_INI} ${_WORKDIR}'")
		EXECUTE_PROCESS(
			COMMAND ${VPLAN_DOIT_EXECUTABLE} ${_INI} ${_WORKDIR}
			WORKING_DIRECTORY ${_WORKDIR}
			RESULT_VARIABLE _DOIT_RES
			ERROR_VARIABLE  _DOIT_STDERR
		)
		IF( NOT _DOIT_RES EQUAL 0 )
			FATAL_ERROR( "ADD_VPLAN_APP(): DOIT failed with message '${_DOIT_STDERR}'. Bailing out." )
		ENDIF()

		# Compile and link generated sources
		SET( _SOURCES ${_DOIT_STDERR} ) # On success, DOIT prints a list of generated sources to stderr
		SET_SOURCE_FILES_PROPERTIES( ${_SOURCES} PROPERTIES GENERATED ON COMPILER_FLAGS ${CS_F77_EXTRA} )

		PREPROCESS_FORTRAN( ${_LIB}_PP_SOURCES "${_SOURCES}" ${PPFLAGS} )
		ADD_LIBRARY( ${_LIB} SHARED ${_WORKDIR}/problem.cc ${${_LIB}_PP_SOURCES} )
		SET_TARGET_PROPERTIES( ${_LIB} PROPERTIES LINKER_LANGUAGE CXX )
# 		INSTALL( TARGETS ${_LIB} DESTINATION ${INSTALL_LIBRARY_DIR} )

		# Mark generated files so that they are removed by 'make clean'
		FILE( GLOB_RECURSE _DOIT_FILES "${_WORKDIR}" ${_WORKDIR}/* )
		SET_DIRECTORY_PROPERTIES( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_DOIT_FILES}" )

	ENDIF()
ENDFUNCTION( ADD_VPLAN_APP )


# ADD_VPLAN_APPLICATION_LIBRARY( <INIFILE> <LIBRARY> )
#
# Create a problem library for VPLAN.
#
# Requirements:
# * The VPLAN package has been found ( by FIND_PACKAGE(VPLAN) ) and used (by USE_VPLAN).
#
# Calls VPLAN's 'doit' to generate derivatives required by the VPLAN problem description file
# <INIFILE>. From the generated sources the library target <LIBRARY> is created.
#
# The library name <LIBRARY> is used to create a meaningful subdirectory for intermediate files
# created during derivative generation. These intermediate files can be removed by building they
# the target 'clean<LIBRARY>'. They are also deleted when the global 'clean' target it built.
MACRO( ADD_VPLAN_APPLICATION_LIBRARY _AVAL_LIB _AVAL_INI )
	MESSAGE( STATUS  "Building VPLAN problem library '${_AVAL_LIB}'." )

	# Check preconditions
	SET( _AVAL_VALID_ARGS TRUE )

	IF( "${_AVAL_LIB}" STREQUAL ""  )
		ERROR( "ADD_VPLAN_APPLICATION_LIBRARY(): empty library name." )
		SET( _AVAL_VALID_ARGS FALSE )
	ENDIF()
	IF( NOT EXISTS "${_AVAL_INI}" )
		ERROR( "ADD_VPLAN_APPLICATION_LIBRARY(): cannot read from ini file '${_AVAL_INI}'." )
		SET( _AVAL_VALID_ARGS FALSE )
	ENDIF()
	IF( NOT VPLAN_FOUND )
		ERROR( "ADD_VPLAN_APPLICATION_LIBRARY(): Required package VPLAN not found." )
		SET( _AVAL_VALID_ARGS FALSE )
	ENDIF()

	# Create working dir for generated files
	SET( _AVAL_WORKDIR "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${_AVAL_LIB}_doit_work" )
	FILE( MAKE_DIRECTORY "${_AVAL_WORKDIR}" )

	# Create problem library
	IF( CMAKE_BUILD_TYPE AND _AVAL_VALID_ARGS )

		# Call doit to generate derivatives
		#MESSAGE( STATUS  "Calling doit. Command: '${VPLAN_DOIT_EXECUTABLE} ${_AVAL_INI} ${_AVAL_WORKDIR}'")
		EXECUTE_PROCESS(
			COMMAND ${VPLAN_DOIT_EXECUTABLE} ${_AVAL_INI} ${_AVAL_WORKDIR}
			WORKING_DIRECTORY ${_AVAL_WORKDIR}
			RESULT_VARIABLE _DOIT_RES
			ERROR_VARIABLE  _DOIT_STDERR
		)
		IF( NOT _DOIT_RES EQUAL 0 )
			FATAL_ERROR( "DOIT failed with message '${_DOIT_STDERR}'. Bailing out." )
		ENDIF()

		# Compile and link generated sources
		SET( _AVAL_SOURCES ${_DOIT_STDERR} ) # On success, DOIT prints a list of generated sources to stderr
		SET_SOURCE_FILES_PROPERTIES( ${_AVAL_SOURCES} PROPERTIES GENERATED ON COMPILER_FLAGS ${CS_F77_EXTRA} )

		PREPROCESS_FORTRAN( ${_AVAL_LIB}_PP_SOURCES "${_AVAL_SOURCES}" )
		ADD_LIBRARY( ${_AVAL_LIB} ${_AVAL_WORKDIR}/problem.cc ${${_AVAL_LIB}_PP_SOURCES} )
		SET_TARGET_PROPERTIES( ${_AVAL_LIB} PROPERTIES LINKER_LANGUAGE CXX )
		INSTALL( TARGETS ${_AVAL_LIB} DESTINATION ${INSTALL_LIBRARY_DIR} )

		# Mark generated files so that they are removed by 'make clean'
# TODO: add dependency from global clean target?
		ADD_CUSTOM_TARGET(
			clean${_AVAL_LIB}
			COMMAND rm -rf ${_AVAL_WORKDIR}
			COMMENT "Cleaning up files from VPLAN application '${_AVAL_LIB}'"
		)
	ENDIF()
ENDMACRO( ADD_VPLAN_APPLICATION_LIBRARY )

ENDIF()
