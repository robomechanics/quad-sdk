# File:   UseADIFOR2.cmake
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
# Including this file makes prepares your CMake script for package XXX. Usage example:
#
# FIND_PACKAGE( XXX ) # invokes FindADIFOR2.cmake
# IF( XXX_FOUND )
#   INCLUDE( UseXXX.cmake )
# ENDIF()
#
####################################################################################################

IF( NOT NETCDF_USED )
	SET( NETCDF_USED TRUE )
	SET ( HAS_NETCDF TRUE )
	IF ( NETCDF4_FOUND )
			SET ( HAS_NETCDF4 TRUE )
	ENDIF()


	ADD_DEFINITIONS( ${NETCDF_COMPILER_FLAGS} )
	INCLUDE_DIRECTORIES( ${NETCDF_INCLUDE_DIRS} )
	LINK_DIRECTORIES( ${NETCDF_LIBRARY_DIRS} )

	MESSAGE( STATUS  "Using NETCDF" )

ENDIF()
