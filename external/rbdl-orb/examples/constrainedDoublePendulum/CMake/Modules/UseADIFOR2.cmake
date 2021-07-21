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

IF( NOT _USEADIFOR2_ )
	SET( _USEADIFOR2_ TRUE )


	ADD_DEFINITIONS( ${ADIFOR2_COMPILER_FLAGS} )
	INCLUDE_DIRECTORIES( ${ADIFOR2_INCLUDE_DIRS} )
	LINK_DIRECTORIES( ${ADIFOR2_LIBRARY_DIRS} )

	SET( ADIFOR2_AD_HOME ${ADIFOR2_DIR}/ADIFOR2.0  )
	SET( ADIFOR2_AD_LIB  ${ADIFOR2_DIR}/ADIFOR2.0.lib )
	SET( ADIFOR2_AD_ARCH "linux" )
	SET( ADIFOR2_AD_OS "Linux" )

	MESSAGE( STATUS  "Using ADIFOR2" )

ENDIF()
