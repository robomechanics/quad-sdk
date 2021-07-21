# File:   InstallCmakeModules.cmake
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
# Install all SimOpt CMake Modules to make them available also for non-SimOpt packages.
#
####################################################################################################

IF( NOT _INSTALLCMAKEMODULES_ )
SET( _INSTALLCMAKEMODULES_ TRUE )

INCLUDE( BuildPaths )

FOREACH( CMAKE_MODULES_FOLDER "CMake/Modules" "cmake/modules" )
	IF( IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${CMAKE_MODULES_FOLDER} )
		INSTALL(
			DIRECTORY ${CMAKE_MODULES_FOLDER}/
			DESTINATION ${INSTALL_CMAKE_DIR}/modules
			PATTERN ".svn" EXCLUDE
			PATTERN ".bak" EXCLUDE
			PATTERN ".BAK" EXCLUDE
			PATTERN "~" EXCLUDE
			PATTERN "CMakeLists.txt" EXCLUDE
		)
	ENDIF()
ENDFOREACH( CMAKE_MODULES_FOLDER )

ENDIF()
