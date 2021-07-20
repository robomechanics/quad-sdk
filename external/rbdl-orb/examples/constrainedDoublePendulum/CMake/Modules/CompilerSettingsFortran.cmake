# File:   CompilerSettingsFortran.cmake
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
# Definees settings for compiling Fortran sources. 
# The settings are stored in variables containing gcc-compatible argument lists.
#
# NOTE: When building fortran, preprocessing and compilation is ususally done in two steps,
#       hence there is NO variable combining settings for preprocessor and compiler
#
# The settings are stored in strings, the single options separated by whitespace.
# They can be passed to the compiler via ADD_DEFINTIONS("${SETTINGS_VAR}") or
# SET_TARGET_PROPERTIES( <TARGET> COMPILER_FLAGS "${SETTINGS_VAR}" ).
#
# The following list of settings are defined:
#   - CS_F77_DIAG_GENERAL        general dignostics, to be used always
#   - CS_F77_DIAG_COMPILER       compiler-specific basic diagnostics, to be used always
#   - CS_F77_TUNING_COMPILER     compiler-specific tuning and optimization, to be used always
#   - CS_F77                     all the previous settings joined, except advanced compiler settings
#   - CS_F77_EXTRA               all the above settings joined
#
#   - CS_F77_DIAG_PREPROCESSOR   preprocessor-specific diagnostics, to be used always
#   - CS_F77_TUNING_PREPROCESSOR preprocessor-specific tuning and optimization, to be used always
#   - CS_F77                     general and all previous preprocessor settings joined
#
# Comments copied from the GCC documentation at http://gcc.gnu.org for GCC version 4.1.2.
# For a detailed description, see there.
#
####################################################################################################

IF( NOT _COMPILERSETTINGSFORTRAN_ )
SET( _COMPILERSETTINGSFORTRAN_ TRUE )

#
# CHECK PRECONDITIONS
#
IF( NOT CMAKE_COMPILER_IS_GNUCC AND NOT CMAKE_COMPILER_IS_GNUCXX )
	ERROR( "Compiler not supported. Currently, we require variants of gcc.")
ENDIF()

#
# Diagnostics: general
# affecting: [p]reprocessor, [c]ompiler or [l]inker
#
SET( CS_F77_DIAG_GENERAL
	-Wall
	# [p] Turns on all optional warnings which are desirable for normal code. At present this is
	# -Wcomment, -Wtrigraphs, -Wmultichar and a warning about integer promotion causing a change of
	# sign in #if	expressions. Note that many of the preprocessor's warnings are on by default and
	# have no options to control them.
	#
	# [c] Enables commonly used warning options that which pertain to usage that we recommend avoiding
	# and that we believe is easy to avoid. This currently includes -Wunused-labels, -Waliasing,
	# -Wsurprising, -Wnonstd-intrinsic and -Wline-truncation.

#	-pedantic
	# [p] Issue all the mandatory diagnostics listed in the C standard. Some of them are left out by
	# default, since they trigger frequently on harmless code.
	#
	# [c] Issue warnings for uses of extensions to FORTRAN 95. -pedantic also applies to C-language
	# constructs where they occur in GNU Fortran source files, such as use of `\e' in a character
	# constant within a directive like `#include'.
	#
	# Valid FORTRAN 95 programs should compile properly with or without this option. However, without
	# this option, certain GNU extensions and traditional Fortran features are supported as well. With
	# this option, many of them are rejected.
	#
	# This should be used in conjunction with -std=std.
)


#
# Diagnostics: preprocessor
#
SET( CS_F77_DIAG_PREPROCESSOR
	-Wcomments
	#Warn whenever a comment-start sequence `/*' appears in a `/*' comment, or whenever a
	#backslash-newline appears in a `//' comment. (Both forms have the same effect.)

	-Wtrigraphs
	# Most trigraphs in comments cannot affect the meaning of the program. However, a trigraph that
	# would	form an escaped newline (`??/' at the end of a line) can, by changing where the comment
	# begins or	ends. Therefore, only trigraphs that would form escaped newlines produce warnings
	# inside a comment.
	#
	# This option is implied by -Wall. If -Wall is not given, this option is still enabled unless
	# trigraphs are enabled. To get trigraph conversion without warnings, but get the other -Wall
	# warnings, use `-trigraphs -Wall -Wno-trigraphs'.

# 	-Wtraditional
	# Warn about certain constructs that behave differently in traditional and ISO C. Also warn about
	# ISO	C constructs that have no traditional C equivalent, and problematic constructs which should
	# be avoided.

	-Wimport
	# Warn the first time `#import' is used.

	-Wundef
	# Warn whenever an identifier which is not a macro is encountered in an `#if' directive, outside
	# of `defined'. Such identifiers are replaced with zero.

	-Wunused-macros
	# Warn about macros defined in the main file that are unused. A macro is used if it is expanded or
	# tested for existence at least once. The preprocessor will also warn if the macro has not been
	# used at the time it is redefined or undefined.
	#
	# Built-in macros, macros defined on the command line, and macros defined in include files are not
	# warned about.
	#
	# Note: If a macro is actually used, but only used in skipped conditional blocks, then F77 will
	# report it as unused. To avoid the warning in such a case, you might improve the scope of the
	# macro's definition by, for example, moving it into the first skipped block. Alternatively, you
	# could provide a dummy use with something like:
	# 	#if defined the_macro_causing_the_warning
	# 	#endif

	-Wendif-labels
	# Warn whenever an `#else' or an `#endif' are followed by text. This usually happens in code of
	# the	form
	#
	# 	#if FOO
	# 	...
	# 	#else FOO
	# 	...
	# 	#endif FOO
	#
	# The second and third FOO should be in comments, but often are not in older programs. This
	# warning is on by default.

# 	-Wsystem-headers
	# Issue warnings for code in system headers. These are normally unhelpful in finding bugs in your
	# own	code, therefore suppressed. If you are responsible for the system library, you may want to
	# see them.
)


#
# Diagnostics: compiler
#
SET( CS_F77_DIAG_COMPILER
	-W
	# Turns on “extra warnings” and, if optimization is specified via -O, the -Wuninitialized option.
	# (This might change in future versions of gfortran)

	-Waliasing
	# Warn about possible aliasing of dummy arguments. The following example will trigger the warning
	# as it would be illegal to bar to modify either parameter.
	#   INTEGER A
	#   CALL BAR(A,A)

#	-Wconversion # deactivated as hits to oftern in 3rd party code
	# Warn about implicit conversions between different types.

#	-Wimplicit-interface # deactivated as hits to oftern in 3rd party code
	# Warn about when procedure are called without an explicit interface. Note this only checks that
	# an explicit interface is present. It does not check that the declared interfaces are consistent
	# across program units.

	-Wsurprising
	# Produce a warning when “suspicious” code constructs are encountered. While technically legal
	# these usually indicate that an error has been made.
	# This currently produces a warning under the following circumstances:
	# * An INTEGER SELECT construct has a CASE that can never be matched as its lower value is
	#   greater than its upper value.
	# * A LOGICAL SELECT construct has three CASE statements.

	-Wunderflow
	# Produce a warning when numerical constant expressions are encountered, which yield an UNDERFLOW
	# during compilation.

#	-Wunused-labels
	# Warn whenever a label is defined but never referenced.
)


#
# Tuning: configure the C preprocessor so it can handle fortran code properly
# (However, it will still NOT remove fortran comments and C++-style line comments!)
#
SET( CS_F77_TUNING_PREPROCESSOR
	-P
	# [p] Inhibit generation of linemarkers in the output from the preprocessor. This might be useful when
	# running the preprocessor on something that is not C code, and will be sent to a program which
	# might be confused by the linemarkers.

	-traditional
	# [p] Try to imitate the behavior of old-fashioned C preprocessors, as opposed to ISO C preprocessors.
	# (equal to -traditional-cpp )

	-x c
	# [p] Select the expected programming language
)


#
# Tuning
# affecting: [p]reprocessor, [c]ompiler or [l]inker
#
SET( CS_F77_TUNING_COMPILER
	-fno-automatic
	# [c]

	-fno-second-underscore
	# [c]
)


IF ( NOT BUILD_32BIT )
	SET (CS_F77_TUNING_COMPILER
		${CS_F77_TUNING_COMPILER}

		# make INTEGER default to 64 bit width on 64 bit platforms
		# i.e. make INTEGER behave like C's 'long' instead of like 'int'.
		-fdefault-integer-8
	)
ENDIF()

SET( CS_F77
	${CS_F77_DIAG_GENERAL}
	${CS_F77_DIAG_COMPILER}
	${CS_F77_TUNING_COMPILER}
	)

SET(CS_PP_F77
	${CS_F77_DIAG_PREPROCESSOR}
	${CS_F77_TUNING_PREPROCESSOR}
)

SET( CS_F77_EXTRA
	${CS_F77}
	${CS_F77_DIAG_COMPILER_EXTRA}
)

# Turn semicolon-separated lists into whitespace separated lists
STRING( REPLACE ";" " " CS_F77_DIAG_GENERAL "${CS_F77_DIAG_GENERAL}" )
STRING( REPLACE ";" " " CS_F77_DIAG_PREPROCESSOR "${CS_F77_DIAG_PREPROCESSOR}" )
STRING( REPLACE ";" " " CS_F77_DIAG_COMPILER "${CS_F77_DIAG_COMPILER}" )
STRING( REPLACE ";" " " CS_F77_TUNING_PREPROCESSOR "${CS_F77_TUNING_PREPROCESSOR}" )
STRING( REPLACE ";" " " CS_F77_TUNING_COMPILER "${CS_F77_TUNING_COMPILER}" )
STRING( REPLACE ";" " " CS_F77_DIAG_COMPILER_EXTRA "${CS_F77_DIAG_COMPILER_EXTRA}" )

STRING( REPLACE ";" " " CS_PP_F77 "${CS_PP_F77}" )
STRING( REPLACE ";" " " CS_F77 "${CS_F77}" )
STRING( REPLACE ";" " " CS_F77_EXTRA "${CS_F77_EXTRA}" )

ENDIF()
