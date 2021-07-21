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
# Convenience macros for writing more readable cmake scripts.
#
####################################################################################################

IF( NOT _LOGGING_ )
SET( _LOGGING_ TRUE )


OPTION( CMAKE_CONFIGURE_VERBOSE
	"Amount of log output at configuration time. ON: verbose, OFF(default): minimal."
	ON
)
MARK_AS_ADVANCED( CMAKE_CONFIGURE_VERBOSE )


# MESSAGE( STATUS  message )
#
# Print the string 'message' as log info.
MACRO( LOG _MSG )
	IF( CMAKE_CONFIGURE_VERBOSE )
		MESSAGE( STATUS "${_MSG}" )
	ENDIF()
ENDMACRO( LOG )

# LOG_FIND( package component )
#
#
# Log that search action for "component" of "package" has started.
MACRO( LOG_FIND _PACKAGE _COMPONENT )
	IF( CMAKE_CONFIGURE_VERBOSE )
# 		IF( NOT "${${_PACKAGE}_FIND_QUIETLY}" )
			IF( NOT ${_COMPONENT} STREQUAL "" )
				MESSAGE( STATUS "Looking for ${_PACKAGE}: ${_COMPONENT}..." )
			ELSE( NOT ${_COMPONENT} STREQUAL "" )
				MESSAGE( STATUS "Looking for ${_PACKAGE}..." )
			ENDIF()
# 		ENDIF()
	ENDIF()
ENDMACRO( LOG_FIND )


# LOG_FOUND( package component success )
#
#
# Log that search action for "component" of "package" has finished with success or not,
# depending on the truth of "success".
MACRO( LOG_FOUND _PACKAGE _COMPONENT _DECIDE_VAR )
	IF( CMAKE_CONFIGURE_VERBOSE )
# 		IF( NOT "${${_PACKAGE}_FIND_QUIETLY}" )
			IF( ${_DECIDE_VAR} )
				IF( NOT ${_COMPONENT} STREQUAL "" )
					MESSAGE( STATUS "Looking for ${_PACKAGE}: ${_COMPONENT}... - found (${${_DECIDE_VAR}})" )
				ELSE( NOT ${_COMPONENT} STREQUAL "" )
					MESSAGE( STATUS "Looking for ${_PACKAGE}... - found (${${_DECIDE_VAR}})" )
				ENDIF()
			ELSE( ${_DECIDE_VAR} )
				IF( NOT ${_COMPONENT} STREQUAL "" )
					MESSAGE( STATUS "Looking for ${_PACKAGE}: ${_COMPONENT}... - NOT found" )
				ELSE( NOT ${_COMPONENT} STREQUAL "" )
					MESSAGE( STATUS "Looking for ${_PACKAGE}... - NOT found" )
				ENDIF()
			ENDIF()
# 		ENDIF()
	ENDIF()
ENDMACRO( LOG_FOUND )

ENDIF()

