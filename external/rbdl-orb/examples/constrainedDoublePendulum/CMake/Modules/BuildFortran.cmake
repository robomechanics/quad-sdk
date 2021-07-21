# File:   BuildFortran.cmake
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
# Defines aux macros for building Fortran files
#
####################################################################################################

IF( NOT _BUILDFORTRAN_ )
SET( _BUILDFORTRAN_ TRUE )

INCLUDE( ProblemHandling )

####################################################################################################

# PREPROCESS_FORTRAN( <OUT_VAR> INPUT_SOURCES [FLAG1] [FLAG2] ... )
#
# Runs a C preprocessor over fortran sources and removes fortran line-comments. The preprocessed
# files are generated in the CMake binary dir, their names are generated internally. The resulting
# file names are stored in the variable OUT_VAR.
#
# Arguments:
#   - OUT_VAR (out): Name of a variable holding the names of the preprocessed files.
#     The name ouf the output variable is internally used to generate target names. As they must be
#     unique, different calls to PREPROCESS_FORTRAN() must use different variable names!
#   - INPUT_SOURCES (in): list of fortran source files
#   - FLAGi (in):         a flag passed to the preprocessor
#
# Typical usage:
#   SET( F_SOURCES file1.F file2.f file3.F )
#   SET( IFLAG "-Imy_include_path/" )
#   PREPROCESS_FORTRAN( PP_F_SOURCES "${F_SOURCES}" "${IFLAG}" )
#   ADD_LIBRARY( libA ${PP_F_SOURCES} )
#
# WARNING: LINUX-specific! (uses sed)
# WARNING: gcc-specific!
MACRO( PREPROCESS_FORTRAN _PPF_OUTVAR _PPF_INPUT_SOURCES )
	# Check preconditions
	IF( "${_PPF_INPUT_SOURCES}" STREQUAL ""  )
		WARNING( "PREPROCESS_FORTRAN() called with no input sources. This usually indicated something is wrong with your CMakeLists.txt file." )
	ENDIF()
	IF( "${_PPF_OUTVAR}" STREQUAL ""  )
		FATAL_ERROR( "PREPROCESS_FORTRAN() called with invalid output variable name." )
	ENDIF()

	# Evaluate optional arguments (preprocessor flags)
	SET( _PPF_FLAGS "" )
	FOREACH( ARG  ${ARGN} ) # cycle over all arguments
		SEPARATE_ARGUMENTS( ARG )
		FOREACH( SUB_ARG ${ARG} ) # cycle over all elements of a list-type (a;b;c) argument
			SET( _PPF_FLAGS ${_PPF_FLAGS} ${SUB_ARG} )
		ENDFOREACH( SUB_ARG )
	ENDFOREACH( ARG )

	# Create working dir: unique directory names required, hence use name of the output var
	SET( _PPF_WORKDIR "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/PPF_${_PPF_OUTVAR}" )
	FILE( MAKE_DIRECTORY "${_PPF_WORKDIR}" )

	# Iterate over all source files and process them separately
	FOREACH( _PPF_INPUT_SOURCE ${_PPF_INPUT_SOURCES} )
		# Prepare required dir names and file names
		GET_FILENAME_COMPONENT( _PPF_INPUT_SOURCE_EXT  ${_PPF_INPUT_SOURCE} EXT )
		GET_FILENAME_COMPONENT( _PPF_INPUT_SOURCE_BASE ${_PPF_INPUT_SOURCE} NAME_WE )

		# Create absulute file paths
		IF( NOT "${_PPF_INPUT_SOURCE}" MATCHES "^/.*" )
			SET( _PPF_INPUT_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/${_PPF_INPUT_SOURCE}" )
		ENDIF()

		# Call C preprocessor of file extension is "F" or "ff"
		SET( _PPF_PP_SOURCE "${_PPF_WORKDIR}/${_PPF_OUTVAR}_${_PPF_INPUT_SOURCE_BASE}.f" )
		ADD_CUSTOM_COMMAND(
			OUTPUT  ${_PPF_PP_SOURCE}
			DEPENDS ${_PPF_INPUT_SOURCE}
			COMMAND
				# Call the C preprocessor to resolve macros, includes and remove C comments
				${CMAKE_C_COMPILER} -E -P -x c -traditional -traditional-cpp
					-I${CMAKE_CURRENT_SOURCE_DIR} ${_PPF_FLAGS} ${_PPF_INPUT_SOURCE}
				# Convert the source to utf-8 (else sed gets problems with umlauts)
				| iconv -f ISO_8859-1 -t utf-8
				# Call sed to strip fortran comments from file (which were ignored by the C preprocessor)
				| sed --silent -e s/^C.*//g -e s/^c.*//g -e s/^!.*//g -e w${_PPF_PP_SOURCE}
			COMMENT "Preprocessing Fortran source ${_PPF_INPUT_SOURCE}"
			VERBATIM
		)
		SET_SOURCE_FILES_PROPERTIES( ${_PPF_PP_SOURCE} PROPERTIES GENERATED ON )
		SET_DIRECTORY_PROPERTIES( PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${_PPF_PP_SOURCE} )

		SET( ${_PPF_OUTVAR} ${${_PPF_OUTVAR}} ${_PPF_PP_SOURCE} )
	ENDFOREACH( _PPF_INPUT_SOURCE )
ENDMACRO( PREPROCESS_FORTRAN )

ENDIF()
