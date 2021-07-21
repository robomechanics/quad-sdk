# File:   SimOptDefaults.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of the MUSCOD/VPLAN suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2007--2009. All rights reserved.
#
####################################################################################################
#
# Convenience file, includes cmake files used by default in the SimOpt group
#
####################################################################################################

IF( NOT _SIMOPTDEFAULTS_ )
	SET( _SIMOPTDEFAULTS_ TRUE )

	#### BASICS
	INCLUDE( Logging )         # Adds macro definitions for logging
	INCLUDE( ProblemHandling ) # Adds macro definitions for problem handling
	INCLUDE( Cosmetics )       # Adds macro definitions and user options for cleaning up the ccmake GUI

	#### PACKAGE MANAGEMENT
	INCLUDE( DefaultSearchPaths ) # Adds variables with default search paths for libs, includes etc.

	#### COMPILER
	INCLUDE( CompilerSettingsFortran ) # Adds variables with flags for the gfortran compiler
	INCLUDE( CompilerSettingsC )       # Adds variables with flags for the gcc compiler
	INCLUDE( CompilerSettingsCpp )     # Adds variables with flags for the g++ compiler
	INCLUDE( CompilerOptions )         # Adds user options affecting the compile and link process
	# Modifies CMAKE's compiler-related variables to activate certain compiler-specific optimizations
	# INCLUDE( OptimizeCompilerSettings )

	#### BUILD
	IF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
		SET( CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR} CACHE PATH "Install directory" FORCE )
	ENDIF()
	INCLUDE( Build )                   # Adds build-related convenience macros
	INCLUDE( BuildPaths )              # Adds variables holding install / bin paths
	INCLUDE( BuildFortran )            # Adds Fortran build-related convenience macros
	MARK_AS_ADVANCED(
		CMAKE_INSTALL_PREFIX
		EXECUTABLE_OUTPUT_PATH
		LIBRARY_OUTPUT_PATH
	)


	#### Package handling
	INCLUDE( CMakeExportBuildSettings )
	INCLUDE( ExportIncludeDirs )
	INCLUDE( ImportIncludeDirs )
	INCLUDE( InstallCmakeModules )

	#### MISC
	MARK_AS_ADVANCED( CMAKE_BACKWARDS_COMPATIBILITY )

ENDIF()
