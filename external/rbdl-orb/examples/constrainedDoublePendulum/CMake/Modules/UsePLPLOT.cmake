# File:   UsePLPLOT.cmake
# Author: Jan Albersmeyer
# Date:   2009
#
#
# Copyright (C) 2009. All rights reserved.
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

IF( NOT _USEPLPLOT_ )
	SET( _USEPLPLOT_ TRUE )
	SET( HAS_PLPLOT TRUE )


	ADD_DEFINITIONS( ${PLPLOT_COMPILER_FLAGS} )
	INCLUDE_DIRECTORIES( ${PLPLOT_INCLUDE_DIRS} )
	LINK_DIRECTORIES( ${PLPLOT_LIBRARY_DIRS} )

	MESSAGE( STATUS  "Using PLPLOT" )

ENDIF()
