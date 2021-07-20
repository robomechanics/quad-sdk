# File:   FindOOQP.cmake
# Author: Martin Felis
# Date:   2009
#
# This file is part of proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Find the OOQP Package
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   OOQP_CONFIG          - Full path of the CMake config file of a OOQP installation
#   OOQP_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

MESSAGE( STATUS "Looking for OOQP" "" )

IF( OOQP_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( OOQP_DIR )
ENDIF()

IF( OOQP_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( OOQP ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( OOQP_FOUND )
	MESSAGE( STATUS "Looking for OOQP (${OOQP_CONFIG})" )
	MARK_AS_ADVANCED( OOQP_DIR )
ENDIF()
