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

IF( NOT _USEX11_ )
	SET( _USEX11_ TRUE )


	ADD_DEFINITIONS( ${X11_COMPILER_FLAGS} )
	INCLUDE_DIRECTORIES( ${X11_INCLUDE_DIR} )
	FOREACH( LIB ${X11_LIBRARIES} )
		GET_FILENAME_COMPONENT( LIB_DIR ${LIB} PATH )
		LINK_DIRECTORIES( ${LIB_DIR} )
	ENDFOREACH( LIB )

	MESSAGE( STATUS  "Using X11" )

ENDIF()
