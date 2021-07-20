# File:   UseQPOASES.cmake
# Author: Jan Albersmeyer
# Date:   2009
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

IF( NOT QPOASES_USED )
	SET( QPOASES_USED TRUE )
	SET( HAS_QPOASES TRUE )


	ADD_DEFINITIONS( ${QPOASES_COMPILER_FLAGS} )
	INCLUDE_DIRECTORIES( ${QPOASES_INCLUDE_DIRS} )
	LINK_DIRECTORIES( ${QPOASES_LIBRARY_DIRS} )

	MESSAGE( STATUS  "Using QPOASES" )

ENDIF()
