# File:   CompilerOptions.cmake
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
# Defines some ccmake user options for some frequently changed compiler options
#
# Comments copied from the GCC documentation at http://gcc.gnu.org for GCC version 4.1.2.
# For a detailed description, see there.
#
# TODO:
# -- diagnostic options for linker?
#
####################################################################################################

IF( NOT _COMPILEROPTIONS_ )
SET( _COMPILEROPTIONS__ TRUE )

#
# CHECK PRECONDITIONS
#
IF( NOT CMAKE_COMPILER_IS_GNUCC AND NOT CMAKE_COMPILER_IS_GNUCXX )
	ERROR( "Compiler not supported. Currently, we require variants of gcc.")
ENDIF()


#
# Build related options
#
OPTION( DIAG_INHIBIT_WARNINGS "Choose if all compiler warnings shall be suppressed." OFF )
OPTION( DIAG_WARNINGS_AS_ERRORS "Choose if compilation warnings shall be treated as errors." OFF )
OPTION( DIAG_STOP_ON_FIRST_ERROR
	"Choose if compilation shall stop after the first occurred error." OFF )
OPTION( DIAG_ONE_LINE_ERRORS
	"ON: Let gcc generate one line error msg, which is useful when using eclipse, Default: ON"
	ON
)

OPTION( BUILD_32BIT
	"Choose of compilation shall be forced to use 32bit, even on 64bit systems." OFF )
OPTION( BUILD_GNU_PROFILER_INFO
	"Choose if extra information for a profiler shall be generated." OFF )
OPTION( BUILD_GDB_DEBUG_INFO
	"Choose if specific debugging information for use by GDB shall be generated (default: ON)." ON )

#
# [1] Diagnostics: general
# affecting: [p]reprocessor, [c]ompiler or [l]inker
#

IF( DIAG_INHIBIT_WARNINGS )
	ADD_DEFINITIONS( -w ) # [p,c,l] Suppress all all warnings
ENDIF()

IF( DIAG_WARNINGS_AS_ERRORS )
	ADD_DEFINITIONS( -Werror ) # [p,c,l] Make all warnings into errors
ENDIF()

IF( DIAG_STOP_ON_FIRST_ERROR )
	ADD_DEFINITIONS( -Wfatal-errors ) # [p,c,l] Stop compilation of the first occurred error
ENDIF()

IF ( DIAG_ONE_LINE_ERRORS )
	IF( CMAKE_COMPILER_IS_GNUCC )
		SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0" )
	ENDIF()
	IF( CMAKE_COMPILER_IS_GNUCXX )
  		SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0" )
	ENDIF()
ENDIF()


#
# [2] Compatibility
# affecting: [p]reprocessor, [c]ompiler or [l]inker
#
IF( BUILD_32BIT )
	# Compile everything with 32bit: BUILD_32BIT must be set on 32bit systems and -m32 does no harm,
	# BUILD_32BIT is optional on 64bit machines and here -m32 important!
	ADD_DEFINITIONS( -m32 )
ELSE ( BUILD_32BIT )
	SET( LIST CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -fdefault-integer-8" )
	SET( LIST CMAKE_Fortran_FLAGS_DEBUG "${CMAKE_Fortran_FLAGS_DEBUG} -fdefault-integer-8" )
	SET( LIST CMAKE_Fortran_FLAGS_RELEASE "${CMAKE_Fortran_FLAGS_RELEASE} -fdefault-integer-8" )
	SET( LIST CMAKE_Fortran_FLAGS_RELWITHDEBINFO "${CMAKE_Fortran_FLAGS_RELWITHDEBINFO} -fdefault-integer-8" )
	SET( LIST CMAKE_Fortran_FLAGS_MINSIZEREL "${CMAKE_Fortran_FLAGS_MINSIZEREL} -fdefault-integer-8" )
ENDIF()

#
# [3] Debugging
#

IF( BUILD_PROFILER_INFO )
	# [c,l] Generate extra code to write profile information suitable for the analysis program gprof.
	# You must use this option when compiling the source files you want data about, and you must also
	# use it when linking.
	ADD_DEFINITIONS( -pg )
ENDIF()


IF( BUILD_GDB_DEBUG_INFO )
	# [c] Produce debugging information for use by GDB. This means to use the most expressive format
	# available including GDB extensions if at all possible.
	ADD_DEFINITIONS( -ggdb3 )
ENDIF()

#
# avoids "Cannot find new threads: generic error" on Ubuntu >= 9.10
#

SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread" )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )

ENDIF()
