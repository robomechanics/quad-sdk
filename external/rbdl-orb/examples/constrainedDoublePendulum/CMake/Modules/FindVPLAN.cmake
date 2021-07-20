# File:   FindVPLAN.cmake
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
# Find a VPLAN installation
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   VPLAN_CONFIG          - Full path of the CMake config file of a VPLAN installation
#   VPLAN_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

MESSAGE( STATUS "Looking for VPLAN" "" )

IF( VPLAN_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( VPLAN_DIR )
ENDIF()

IF( VPLAN_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( VPLAN ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( VPLAN_FOUND )
	MESSAGE( STATUS "Looking for VPLAN (${VPLAN_CONFIG})" )
	MARK_AS_ADVANCED( VPLAN_DIR )
ENDIF()
