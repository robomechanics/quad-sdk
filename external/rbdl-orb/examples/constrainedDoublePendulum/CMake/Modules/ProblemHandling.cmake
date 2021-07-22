# File:   ProblemHandling.cmake
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

IF( NOT _PROBLEMHANDLING_ )
SET( _PROBLEMHANDLING_ TRUE )


####################################################################################################
#### DEBUGGING / PROBLEM HANDLING MACROS
####################################################################################################
OPTION( CMAKE_VERBOSE_DEBUG
	"Debug information in CMake scripts printed at configuration time. ON: verbose, OFF(default): minimal."
	ON
)
MARK_AS_ADVANCED( CMAKE_VERBOSE_DEBUG )


# D_MESSAGE( STATUS  <var> )
#
# Debug macro, prints the name of 'var' and its content at configuration time
#
# TODO:
# * output current location
# * does not work: no output of variable content. why?
#
MACRO( D_LOG _D_LOG_MSG )
	IF( CMAKE_VERBOSE_DEBUG )
		MESSAGE( STATUS "${_D_LOG_MSG}" )
	ENDIF()
ENDMACRO( D_LOG )


# WARN( message )
#
# Debug macro, prints the string 'message' as warning.
#
# TODO: output current location
MACRO( WARN _WARN_MESSAGE )
	IF( CMAKE_VERBOSE_DEBUG )
		MESSAGE( STATUS "WARNING: ${_WARN_MESSAGE}" )
	ENDIF()
ENDMACRO( WARN )


# ERROR( message )
#
# Debug macro, prints the string 'message' as error.
#
# TODO: output current location
MACRO( ERROR _ERROR_MESSAGE )
	MESSAGE( SEND_ERROR "${_ERROR_MESSAGE}" )
ENDMACRO( ERROR )


# FATAL_ERROR( message )
#
# Debug macro, prints the string 'message' as FATAL_ERROR.
#
# TODO: output current location
MACRO( FATAL_ERROR FATAL__ERROR_MESSAGE )
	MESSAGE( FATAL_ERROR "${FATAL__ERROR_MESSAGE}" )
ENDMACRO( FATAL_ERROR )

ENDIF()
