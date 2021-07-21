# File:   CompilerSettingsCpp.cmake
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
# Defines settings for compiling C++ sources.
# The settings are stored in variables containing gcc-compatible argument lists.
#
# The settings are stored in strings, the single options separated by whitespace.
# They can be passed to the compiler via ADD_DEFINTIONS("${SETTINGS_VAR}") or
# SET_TARGET_PROPERTIES( <TARGET> COMPILER_FLAGS "${SETTINGS_VAR}" ).
#
# The following list of settings are defined:
#   - CS_CPP_DIAG_GENERAL        general dignostics, to be used always
#   - CS_CPP_DIAG_PREPROCESSOR   preprocessor-specific diagnostics, to be used always
#   - CS_CPP_DIAG_COMPILER       compiler-specific basic diagnostics, to be used always
#   - CS_CPP_DIAG_COMPILER_EXTRA compiler-specific advanced diagnostics, not mandatory
#   - CS_CPP_TUNING_COMPILER     compiler-specific tuning and optimization, to be used always
#
#   - CS_CPP                     all the above settings joined, except advanced compiler settings
#   - CS_CPP_EXTRA               all the above settings joined
#
# Comments copied from the GCC documentation at http://gcc.gnu.org for GCC version 4.1.2.
# For a detailed description, see there.
#
####################################################################################################

IF( NOT _COMPILERSETTINGSCPP_ )
SET( _COMPILERSETTINGSCPP_ TRUE )

#
# CHECK PRECONDITIONS
#
IF( NOT CMAKE_COMPILER_IS_GNUCC AND NOT CMAKE_COMPILER_IS_GNUCXX )
	ERROR( "Compiler not supported. Currently, we require variants of gcc.")
ENDIF()

#
# [1] Diagnostics: general
# affecting: [p]reprocessor, [c]ompiler or [l]inker
#
SET( CS_CPP_DIAG_GENERAL
	-Wall
	# Combines various warnings. They are listed in detail in [3].
	#
	# [p] Turns on all optional warnings which are desirable for normal code.
	# At present this is -Wcomment, -Wtrigraphs, -Wmultichar and a warning about integer promotion
	# causing a change of sign in #if	expressions. Note that many of the preprocessor's warnings are
	# on by default and have no options to control them.
	#
	# [c] Enables commonly used warning options that which pertain to usage that we recommend avoiding
	# and that we believe is easy to avoid. This currently includes -Wunused-labels, -Waliasing,
	# -Wsurprising, -Wnonstd-intrinsic and -Wline-truncation.

	# This is included in Wall, but is horribly annoying when compiling old code, hence we deactivate it here
	-Wno-write-strings

	#-pedantic
	# [p] Issue all the mandatory diagnostics listed in the C standard. Some of them are left out by
	# default, since they trigger frequently on harmless code.
	#
	# [c] Issue all the warnings demanded by strict ISO C and ISO C++; reject all programs that use
	# forbidden extensions, and some other programs that do not follow ISO C and ISO C++. For ISO C,
	# follows the version of the ISO C standard specified by any -std option used.
	#
	# Valid ISO C and ISO C++ programs should compile properly with or without this option (though a
	# rare few will require -ansi or a -std option specifying the required version of ISO C). However,
	# without this option, certain GNU extensions and traditional C and C++ features are supported as
	# well. With this option, they are rejected.
	#
	# -pedantic does not cause warning messages for use of the alternate keywords whose names begin
	# and end with `__'. Pedantic warnings are also disabled in the expression that follows
	# __extension__.	However, only system header files should use these escape routes; application
	# programs should avoid	them. See Alternate Keywords.
)


#
# [2] Diagnostics: preprocessor
#
SET( CS_CPP_DIAG_PREPROCESSOR
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

# 	-Wunused-macros
	# Warn about macros defined in the main file that are unused. A macro is used if it is expanded or
	# tested for existence at least once. The preprocessor will also warn if the macro has not been
	# used at the time it is redefined or undefined.
	#
	# Built-in macros, macros defined on the command line, and macros defined in include files are not
	# warned about.
	#
	# Note: If a macro is actually used, but only used in skipped conditional blocks, then CPP will
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

#	-Wvariadic-macros
	# Warn if variadic macros are used in pedantic ISO C90 mode, or the GNU alternate syntax when in
	# pedantic ISO C99 mode. To inhibit the warning messages, use -Wno-variadic-macros.
)


#
# [3] Diagnostics: compiler
# NOTE: the following `-W' options are combined in -Wall (see [1]).
# They comprise constructions that some users consider questionable, and that are easy to avoid
# (or modify to prevent the warning), even in conjunction with macros. This also enables some
# language-specific warnings.
#
SET( CS_CPP_DIAG_COMPILER_BASIC
# 	-Wchar-subscripts
	# Warn if an array subscript has type char. This is a common cause of error, as programmers often
	# forget that this type is signed on some machines. This warning is enabled by -Wall.

# 	-Wcomment
	# Warn whenever a comment-start sequence `/*' appears in a `/*' comment, or whenever a
	# Backslash-Newline appears in a `//' comment. This warning is enabled by -Wall.

# 	-Wformat
	# Check calls to printf and scanf, etc., to make sure that the arguments supplied have types
	# appropriate to the format string specified, and that the conversions specified in the format string
	# make sense. Since -Wformat also checks for null format arguments for several functions, -Wformat
	# also implies -Wnonnull.-Wformat is included in -Wall.

# 	-Wformat-y2k
	# If -Wformat is specified, also warn about strftime formats which may yield only a two-digit
	# year.

# 	-Winit-self
	# Warn about uninitialized variables which are initialized with themselves. Note this option can
	# only be used with the -Wuninitialized option, which in turn only works with -O1 and above.

# 	-Wmissing-braces
	# Warn if an aggregate or union initializer is not fully bracketed. In the following example, the
	# initializer for `a' is not fully bracketed, but that for `b' is fully bracketed.
	#   int a[2][2] = { 0, 1, 2, 3 };
	#   int b[2][2] = { { 0, 1 }, { 2, 3 } };
	# This warning is enabled by -Wall.

# 	-Wmissing-include-dirs
	# Warn if a user-supplied include directory does not exist.

# 	-Wparentheses
	# Warn if parentheses are omitted in certain contexts, such as when there is an assignment in a
	# context where a truth value is expected, or when operators are nested whose precedence people often
	# get confused about. Only the warning for an assignment used as a truth value is supported when
	# compiling C++; the other warnings are only supported when compiling C.
	# This warning is enabled by -Wall.

# 	-Wsequence-point
	# Warn about code that may have undefined semantics because of violations of sequence point rules
	# in the C standard. Examples of code with undefined behavior are a = a++;, a[n] = b[n++] and
	# a[i++] = i;. The present implementation of this option only works for C programs. A future
	# implementation may also work for C++ programs. This warning is enabled by -Wall.

# 	-Wreturn-type
	# Warn whenever a function is defined with a return-type that defaults to int. Also warn about any
	# return statement with no return-value in a function whose return-type is not void.
	# This warning is enabled by -Wall.

# 	-Wswitch-default
	# Warn whenever a switch statement does not have a default case.

# 	-Wswitch-enum
	# Warn whenever a switch statement has an index of enumerated type and lacks a case for one or
	# more of the named codes of that enumeration. case labels outside the enumeration range also provoke
	# warnings when this option is used.

# 	-Wtrigraphs
	# Warn if any trigraphs are encountered that might change the meaning of the program (trigraphs
	# within comments are not warned about). This warning is enabled by -Wall.

# 	-Wunused-function
	# Warn whenever a static function is declared but not defined or a non-inline static function is
	# unused. This warning is enabled by -Wall.

# 	-Wunused-label
	# Warn whenever a label is declared but not used. This warning is enabled by -Wall.

# 	-Wunused-parameter
	# Warn whenever a function parameter is unused aside from its declaration.

# 	-Wunused-variable
	# Warn whenever a local variable or non-constant static variable is unused aside from its
	# declaration. This warning is enabled by -Wall.

# 	-Wunused-value
	# Warn whenever a statement computes a result that is explicitly not used. This warning is enabled
	# by -Wall.

# 	-Wunused
	# All the above -Wunused options combined.

# 	-Wuninitialized
	# Warn if an automatic variable is used without first being initialized or if a variable may be
	# clobbered by a setjmp call.
	#
	# These warnings are possible only in optimizing compilation, because they require data flow
	# information that is computed only when optimizing. If you don't specify -O, you simply won't get
	# these warnings. This warning is enabled by -Wall.

# 	-Wunknown-pragmas
	# Warn when a #pragma directive is encountered which is not understood by GCC. If this command
	# line option is used, warnings will even be issued for unknown pragmas in system header files. This
	# is not the case if the warnings were only enabled by the -Wall command line option.

# 	-Wstrict-aliasing
	# This option is only active when -fstrict-aliasing is active. It warns about code which might
	# break the strict aliasing rules that the compiler is using for optimization. The warning does not
	# catch all cases, but does attempt to catch the more common pitfalls. It is included in -Wall.

# 	-Wstrict-aliasing=2
	# This option is only active when -fstrict-aliasing is active. It warns about code which might
	# break the strict aliasing rules that the compiler is using for optimization. This warning catches
	# more cases than -Wstrict-aliasing, but it will also give a warning for some ambiguous cases that are
	# safe.

# 	-Wnon-virtual-dtor # (C++ only)
	# Warn when a class appears to be polymorphic, thereby requiring a virtual destructor, yet it
	# declares a non-virtual one. This warning is enabled by -Wall.

# 	-Wreorder # (C++ only)
	# Warn when the order of member initializers given in the code does not match the order in which
	# they must be executed. This warning is enabled by -Wall.
)


# [4] Diagnostics: compiler (not included in -Wall)
# NOTE: the following options are NOT combined within -Wall.
# The following -W... options are not implied by -Wall. Some of them warn about constructions that
# users generally do not consider questionable, but which occasionally you might wish to check for;
# others warn about constructions that are necessary or hard to avoid in some cases, and there is no
# simple way to modify the code to suppress the warning.
SET( CS_CPP_DIAG_COMPILER_EXTRA
	-Wextra
	# Print extra warning messages for various events.

	-Wfloat-equal
	# Warn if floating point values are used in equality comparisons.

	-Wshadow
	# Warn whenever a local variable shadows another local variable, parameter or global variable or
	# whenever a built-in function is shadowed.

	-Wunsafe-loop-optimizations
	# Warn if the loop cannot be optimized because the compiler could not assume anything on the
	# bounds of the loop indices. With -funsafe-loop-optimizations warn if the compiler made such
	# assumptions.

	-Wpointer-arith
	# Warn about anything that depends on the ?size of? a function type or of void. GNU C assigns
	# these types a size of 1, for convenience in calculations with void * pointers and pointers to
	# functions.

	-Wcast-qual
	# Warn whenever a pointer is cast so as to remove a type qualifier from the target type. For
	# example, warn if a const char * is cast to an ordinary char *.

	-Wcast-align
	# Warn whenever a pointer is cast such that the required alignment of the target is increased. For
	# example, warn if a char * is cast to an int * on machines where integers can only be accessed at
	# two- or four-byte boundaries.

	-Wwrite-strings
	# When compiling C, give string constants the type const char[length] so that copying the address
	# of one into a non-const char * pointer will get a warning; when compiling C++, warn about the
	# deprecated conversion from string constants to char *.

	-Wconversion
	# Warn if a prototype causes a type conversion that is different from what would happen to the
	# same argument in the absence of a prototype.

	-Wsign-compare
	# Warn when a comparison between signed and unsigned values could produce an incorrect result when
	# the signed value is converted to unsigned. This warning is also enabled by -Wextra.

	-Wmissing-field-initializers
	# Warn if a structure's initializer has some fields missing.  This warning is included in -Wextra.

	-Wmissing-noreturn
	# Warn about functions which might be candidates for attribute noreturn. Note these are only
	# possible candidates, not absolute ones.

	-Wmissing-format-attribute
	# Warn about function pointers which might be candidates for format attributes. Note these are
	# only possible candidates, not absolute ones.

	-Wno-deprecated-declarations
	# Do not warn about uses of functions, variables, and types marked as deprecated by using the
	# deprecated attribute. (see Function Attributes, see Variable Attributes, see Type Attributes.)

 	-Wredundant-decls
	# Warn if anything is declared more than once in the same scope, even in cases where multiple
	# declaration is valid and changes nothing.

# 	-Wunreachable-code
	# Warn if the compiler detects that code will never be executed. It is possible for this option to
	# produce a warning even though there are circumstances under which part of the affected line can
	# be executed, so care should be taken when removing apparently-unreachable code.

	-Winline
	# Warn if a function can not be inlined and it was declared as inline. Even with this option, the
	# compiler will not warn about failures to inline functions declared in system headers.

	-Wlong-long
	# Warn if `long long' type is used. This is default.

	-Wvolatile-register-var
	# Warn if a register variable is declared volatile. The volatile modifier does not inhibit all
	# optimizations that may eliminate reads and/or writes to register variables.

	-Wdisabled-optimization
	# Warn if a requested optimization pass is disabled.

# 	-Weffc++ # (C++ only)
	# Warn about violations of the following style guidelines from Scott Meyers' Effective C++ book:
	# * Item 11: Define a copy constructor and an assignment operator for classes with dynamically
	#   allocated memory.
	# * Item 12: Prefer initialization to assignment in constructors.
	# * Item 14: Make destructors virtual in base classes.
	# * Item 15: Have operator= return a reference to *this.
	# * Item 23: Don't try to return a reference when you must return an object.
	# Also warn about violations of the following style guidelines:
	#	* Item 6: Distinguish between prefix and postfix forms of increment and decrement operators.
	#	* Item 7: Never overload &&, ||, or ,.
	# When selecting this option, be aware that the standard library headers do not obey all of these
	# guidelines; use `grep -v' to filter out those warnings.

	-Woverloaded-virtual #(C++ only)
	# Warn when a function declaration hides virtual functions from a base class.
)


#
# [5] Tuning
# affecting: [p]reprocessor, [c]ompiler or [l]inker
#
SET( CS_CPP_TUNING_COMPILER
	-felide-constructors
# 	-fno-elide-constructors
	# [c] The C++ standard allows an implementation to omit creating a temporary which is only used to
	# initialize another object of the same type. Specifying this option disables that optimization,
	# and forces G++ to call the copy constructor in all cases.

# 	-fno-rtti
	# [c] Disable generation of information about every class with virtual functions for use by the
	# C++ runtime type identification features (`dynamic_cast' and `typeid'). If you don't use those
	# parts of the language, you can save some space by using this flag.

# 	-fno-exceptions
# 	-fexceptions
	# [c] Enable exception handling. Generates extra code needed to propagate exceptions.
)

SET( CS_CPP
	${CS_CPP_DIAG_GENERAL}
	${CS_CPP_DIAG_PREPROCESSOR}
	${CS_CPP_DIAG_COMPILER}
	${CS_CPP_TUNING_COMPILER}
	)

SET( CS_CPP_EXTRA
	${CS_CPP}
	${CS_CPP_DIAG_COMPILER_EXTRA}
)

# Turn semicolon-separated lists into whitespace separated lists
STRING( REPLACE ";" " " CS_CPP "${CS_CPP}" )
STRING( REPLACE ";" " " CS_CPP_DIAG_GENERAL "${CS_CPP_DIAG_GENERAL}" )
STRING( REPLACE ";" " " CS_CPP_DIAG_PREPROCESSOR "${CS_CPP_DIAG_PREPROCESSOR}" )
STRING( REPLACE ";" " " CS_CPP_DIAG_COMPILER "${CS_CPP_DIAG_COMPILER}" )
STRING( REPLACE ";" " " CS_CPP_TUNING_COMPILER "${CS_CPP_TUNING_COMPILER}" )
STRING( REPLACE ";" " " CS_CPP_EXTRA "${CS_CPP_EXTRA}" )
STRING( REPLACE ";" " " CS_CPP_DIAG_COMPILER_EXTRA "${CS_CPP_DIAG_COMPILER_EXTRA}" )

ENDIF()
