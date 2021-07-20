# File:   FindLIBLAC.cmake
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
# Find a LIBLAC installation
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   LIBLAC_CONFIG          - Full path of the CMake config file of a LIBLAC installation
#   LIBLAC_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

MESSAGE( STATUS "Looking for LIBLAC" "" )

IF( LIBLAC_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( LIBLAC_DIR )
ENDIF()

IF( LIBLAC_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( LIBLAC ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( LIBLAC_FOUND )
	MESSAGE( STATUS "Looking for LIBLAC (${LIBLAC_CONFIG})" )
	MARK_AS_ADVANCED( LIBLAC_DIR )
ENDIF()
