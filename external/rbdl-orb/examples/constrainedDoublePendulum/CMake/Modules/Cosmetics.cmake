# File:   Cosmetics.cmake
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
# Macro definitions and user options for cleaning up the ccmake GUI
#
####################################################################################################

IF( NOT _COSMETICS_ )
SET( _COSMETICS_ TRUE )

# HIDE_FROM_GUI( <var> )
#
# "Hide" variable <var> from the GUI by declaring it as "internal".
# Pure cosmetic effect, does not affect behaviour of the CMake script, as long as <var> is a
# cached variable.
MACRO( HIDE_FROM_GUI _VAR )
	SET( ${_VAR} ${${_VAR}} CACHE INTERNAL "" FORCE )
ENDMACRO( HIDE_FROM_GUI )

# SHOW_STRING_IN_GUI( <var> )
#
# Make an internal variable <var> visible in the GUI by declaring it as cached string variable.
# Pure cosmetic effect, does not affect behaviour of the CMake script, as long as <var> is an
# "internal" variable.
MACRO( SHOW_STRING_IN_GUI _VAR )
	SET( ${_VAR} ${${_VAR}} CACHE STRING "" FORCE )
ENDMACRO( SHOW_STRING_IN_GUI )

# SHOW_BOOL_IN_GUI( <var> )
#
# Make an internal variable <var> visible in the GUI by declaring it as cached bool variable.
# Pure cosmetic effect, does not affect behaviour of the CMake script, as long as <var> is an
# "internal" variable.
MACRO( SHOW_BOOL_IN_GUI _VAR )
	SET( ${_VAR} ${${_VAR}} CACHE BOOL "" FORCE )
ENDMACRO( SHOW_BOOL_IN_GUI )

# SHOW_PATH_IN_GUI( <var> )
#
# Make an internal variable <var> visible in the GUI by declaring it as cached path variable.
# Pure cosmetic effect, does not affect behaviour of the CMake script, as long as <var> is an
# "internal" variable.
MACRO( SHOW_PATH_IN_GUI _VAR )
	SET( ${_VAR} ${${_VAR}} CACHE PATH "" FORCE )
ENDMACRO( SHOW_PATH_IN_GUI )

####################################################################################################

# (De-)Activate only required build types
OPTION( CMAKE_HIDE_SPECIAL_BUILD_TYPES
	"ON: hide build types MinSizeRel and RelWithDebInfo from the GUI. (default: ON)" ON
)

####################################################################################################

IF( CMAKE_HIDE_SPECIAL_BUILD_TYPES )
	HIDE_FROM_GUI( CMAKE_CXX_FLAGS_MINSIZEREL )
	HIDE_FROM_GUI( CMAKE_CXX_FLAGS_RELWITHDEBINFO )
	HIDE_FROM_GUI( CMAKE_CXX_FLAGS )
	HIDE_FROM_GUI( CMAKE_CXX_FLAGS_DEBUG )
	HIDE_FROM_GUI( CMAKE_CXX_FLAGS_RELEASE )
	HIDE_FROM_GUI( CMAKE_C_FLAGS )
	HIDE_FROM_GUI( CMAKE_C_FLAGS_MINSIZEREL )
	HIDE_FROM_GUI( CMAKE_C_FLAGS_RELWITHDEBINFO )
	HIDE_FROM_GUI( CMAKE_EXE_LINKER_FLAGS )
	HIDE_FROM_GUI( CMAKE_EXE_LINKER_FLAGS_MINSIZEREL )
	HIDE_FROM_GUI( CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO )
	HIDE_FROM_GUI( CMAKE_Fortran_FLAGS )
	HIDE_FROM_GUI( CMAKE_Fortran_FLAGS_MINSIZEREL )
	HIDE_FROM_GUI( CMAKE_Fortran_FLAGS_RELWITHDEBINFO )
	HIDE_FROM_GUI( CMAKE_MODULE_LINKER_FLAGS )
	HIDE_FROM_GUI( CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL )
	HIDE_FROM_GUI( CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO )
	HIDE_FROM_GUI( CMAKE_SHARED_LINKER_FLAGS )
	HIDE_FROM_GUI( CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL )
	HIDE_FROM_GUI( CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO )
ELSE( CMAKE_HIDE_SPECIAL_BUILD_TYPES )
	SHOW_STRING_IN_GUI( CMAKE_CXX_FLAGS_MINSIZEREL )
	SHOW_STRING_IN_GUI( CMAKE_CXX_FLAGS_RELWITHDEBINFO )
	SHOW_STRING_IN_GUI( CMAKE_CXX_FLAGS )
	SHOW_STRING_IN_GUI( CMAKE_CXX_FLAGS_DEBUG )
	SHOW_STRING_IN_GUI( CMAKE_CXX_FLAGS_RELEASE )
	SHOW_STRING_IN_GUI( CMAKE_C_FLAGS )
	SHOW_STRING_IN_GUI( CMAKE_C_FLAGS_MINSIZEREL )
	SHOW_STRING_IN_GUI( CMAKE_C_FLAGS_RELWITHDEBINFO )
	SHOW_STRING_IN_GUI( CMAKE_EXE_LINKER_FLAGS )
	SHOW_STRING_IN_GUI( CMAKE_EXE_LINKER_FLAGS_MINSIZEREL )
	SHOW_STRING_IN_GUI( CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO )
	SHOW_STRING_IN_GUI( CMAKE_Fortran_FLAGS )
	SHOW_STRING_IN_GUI( CMAKE_Fortran_FLAGS_MINSIZEREL )
	SHOW_STRING_IN_GUI( CMAKE_Fortran_FLAGS_RELWITHDEBINFO )
	SHOW_STRING_IN_GUI( CMAKE_MODULE_LINKER_FLAGS )
	SHOW_STRING_IN_GUI( CMAKE_MODULE_LINKER_FLAGS_MINSIZEREL )
	SHOW_STRING_IN_GUI( CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO )
	SHOW_STRING_IN_GUI( CMAKE_SHARED_LINKER_FLAGS )
	SHOW_STRING_IN_GUI( CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL )
	SHOW_STRING_IN_GUI( CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO )
ENDIF()

####################################################################################################

MARK_AS_ADVANCED( CMAKE_HIDE_SPECIAL_BUILD_TYPES )

ENDIF()
