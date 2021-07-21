# File:   BuildMuscodApps.cmake
# Author: Christian Hoffmann
# Date:   2007--2008
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
# Defines convenience macros for building MUSCOD applications
#
####################################################################################################

IF( NOT _BUILDMUSCODAPPS_ )
SET( _BUILDMUSCODAPPS_ TRUE )

####################################################################################################

# ADD_MUSCOD_APP( <APP> <SOURCES> <DEPS> [<FLAGS>] )
#
# Convenience macro performing several tasks required for building a MUSCOD application executable
FUNCTION( ADD_MUSCOD_APP _APP _SOURCES _DEPS  )
	IF( ARGC GREATER 3 )
		SET( COMPILE_FLAGS "${ARGV3}" )
	ENDIF()

	# create the executable with required properties
	ADD_LIBRARY( "${_APP}" SHARED ${_SOURCES} )
	SET_TARGET_PROPERTIES( "${_APP}" PROPERTIES
		INSTALL_RPATH_USE_LINK_PATH ON       # use all known link dirs as rpaths
#		BUILD_WITH_INSTALL_RPATH OFF         # to produce working executables without "make install"
#		INSTALL_RPATH ${INSTALL_LIBRARY_DIR} # find also libs from this package
	)
	IF( NOT "${COMPILE_FLAGS}" STREQUAL "" )
		SET_TARGET_PROPERTIES( "${_APP}" PROPERTIES COMPILE_FLAGS "${COMPILE_FLAGS}" )
	ENDIF()
	IF( NOT "${_DEPS}" STREQUAL "" )
		TARGET_LINK_LIBRARIES( "${_APP}" ${_DEPS} )
	ENDIF()
ENDFUNCTION( ADD_MUSCOD_APP )

# LINK_MUSCOD_RESOURCES( <SOURCEDIR> <TARGETDIR>)
# TODO:
# * Do not delete, only link when not already present
# * Check if source exists
FUNCTION( LINK_MUSCOD_RESOURCES _SDIR _TDIR )
	IF( NOT IS_DIRECTORY "${_SDIR}" )
		ERROR( "LINK_MUSCOD_RESOURCES(): Cannot read from '${_SDIR}', not a directory." )
	ENDIF()
	IF( NOT IS_DIRECTORY "${_TDIR}" )
		ERROR( "LINK_MUSCOD_RESOURCES(): Cannot write to '${_TDIR}', not a directory." )
	ENDIF()

	IF( NOT _MLR_COUNTER )
		SET( _MLR_COUNTER 1 )
	ELSE( NOT _MLR_COUNTER )
		MATH( EXPR _MLR_COUNTER ${_MLR_COUNTER} + 1 )
	ENDIF()

	ADD_CUSTOM_TARGET( link_muscod_targets_${_MLR_COUNTER}
		ALL
		COMMAND ${CMAKE_COMMAND} -E remove
			${_TDIR}/DAT
			${_TDIR}/RES
			${_TDIR}/default.plot
		COMMAND ${CMAKE_COMMAND} -E create_symlink ${_SDIR}/DAT ${_TDIR}/DAT
		COMMAND ${CMAKE_COMMAND} -E make_directory ${_TDIR}/RES
		COMMAND ${CMAKE_COMMAND} -E copy ${_SDIR}/default.plot.template ${_TDIR}/default.plot
		COMMENT "Creating symlinks to folders and files required by MUSCOD in ${_SDIR}."
		VERBATIM
	)
ENDFUNCTION( LINK_MUSCOD_RESOURCES )


# PREPARE_MUSCOD_APP_BUILD( <SOURCEDIR> <TARGETDIR>)
FUNCTION( PREPARE_MUSCOD_APP_BUILD _SDIR _TDIR )
	IF( NOT IS_DIRECTORY "${_SDIR}" )
		ERROR( "LINK_MUSCOD_RESOURCES(): Cannot read from '${_SDIR}', not a directory." )
	ENDIF()
	IF( NOT IS_DIRECTORY "${_TDIR}" )
		ERROR( "LINK_MUSCOD_RESOURCES(): Cannot write to '${_TDIR}', not a directory." )
	ENDIF()

	IF( EXISTS "${_SDIR}/DAT" )
		IF( NOT EXISTS "${_TDIR}/DAT" )
			MESSAGE( "Creating link ${_SDIR}/DAT -> ${_TDIR}/DAT" )
			EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND} -E
				create_symlink "${_SDIR}/DAT" "${_TDIR}/DAT" )
		ENDIF()
	ENDIF()

	IF( NOT EXISTS "${_TDIR}/RES" )
		MESSAGE( "Creating ${_TDIR}/RES." )
		EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND} -E make_directory "${_TDIR}/RES" )
	ENDIF()

	IF( EXISTS "${_SDIR}/default.plot.template" )
		IF( NOT EXISTS "${_TDIR}/default.plot" )
			MESSAGE( "Copying ${_SDIR}/default.plot.template -> ${_TDIR}/default.plot" )
			EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND} -E copy
				"${_SDIR}/default.plot.template" "${_TDIR}/default.plot" )
		ENDIF()
	ENDIF()
ENDFUNCTION( PREPARE_MUSCOD_APP_BUILD )

ENDIF()