# File:   FindCOMMON_CODE.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of tproprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Find a COMMON_CODE installation
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   COMMON_CODE_CONFIG          - Full path of the CMake config file of a COMMON_CODE installation
#   COMMON_CODE_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

MESSAGE( STATUS "Looking for COMMON_CODE" "" )

IF( COMMON_CODE_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( COMMON_CODE_DIR )
ENDIF()

IF( COMMON_CODE_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( COMMON_CODE ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( COMMON_CODE_FOUND )
	MESSAGE( STATUS "Looking for COMMON_CODE (${COMMON_CODE_CONFIG})" )
	MARK_AS_ADVANCED( COMMON_CODE_DIR )
ENDIF()
