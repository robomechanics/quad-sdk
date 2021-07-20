# File:   FindXXX.cmake
# Author: Christian Hoffmann
# Date:   2007--2009
#
# This file is part of proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
# Copyright (C) 2007--2009 by the authors. All rights reserved.
#
####################################################################################################
#
# Find a XXX installation
#
# This file fulfils the CMAKE modules guidelines:
# <http://www.cmake.org/cgi-bin/viewcvs.cgi/Modules/readme.txt?root=CMake&view=markup>
#
# PLEASE READ THERE BEFORE CHANGING SOMETHING!
#
# Defines:
#   XXX_CONFIG          - Full path of the CMake config file of a XXX installation
#   XXX_DIR    [cached] - Full path of the dir holding the CMake config file, cached
#
###################################################################################################

INCLUDE( DefaultSearchPaths )

LOG_FIND( "XXX" "" )

IF( XXX_FIND_QUIETLY )
	SET( DEMAND_TYPE QUIET )
	MARK_AS_ADVANCED( XXX_DIR )
ENDIF()

IF( XXX_FIND_REQUIRED )
	SET( DEMAND_TYPE REQUIRED )
ENDIF()

FIND_PACKAGE( XXX ${DEMAND_TYPE} NO_MODULE PATHS ${DEFAULT_PACKAGE_DIRS} )

IF( XXX_FOUND )
	LOG_FOUND( "XXX" "" XXX_CONFIG )
	MARK_AS_ADVANCED( XXX_DIR )
ENDIF()
