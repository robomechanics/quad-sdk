# File:   FindSONIC.cmake
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
# Find a SONIC installation
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   SONIC_CONFIG          - Full path of the CMake config file of a SONIC installation
#   SONIC_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

MESSAGE( STATUS "Looking for SONIC" "" )

IF( SONIC_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( SONIC_DIR )
ENDIF()

IF( SONIC_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( SONIC ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( SONIC_FOUND )
	MESSAGE( STATUS "Looking for SONIC (${SONIC_CONFIG})" )
	MARK_AS_ADVANCED( SONIC_DIR )
ENDIF()
