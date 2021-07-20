# File:   ImportIncludeDirs.cmake
# Author: Christian Hoffmann
# Date:   2009
#
# This file is part of the MUSCOD/VPLAN suite, which is proprietary software of
#   Simulation and Optimization Workgroup
#   Interdisciplinary Center for Scientific Computing (IWR)
#   University of Heidelberg, Germany.
#
# Copyright (C) 2009. All rights reserved.
#
####################################################################################################
#
# TODO: doc
#
####################################################################################################

IF( NOT _IMPORTINCLUDEDIRS_ )
	SET( _IMPORTINCLUDEDIRS_ TRUE )

	INCLUDE( ProblemHandling )

	FUNCTION( IMPORT_INCLUDE_DIRS SFILE )

		# Check arguments
		IF( NOT EXISTS ${SFILE} )
			FATAL_ERROR( "IMPORT_INCLUDE_DIRS called with invalid source file '${SFILE}'.")
		ELSE()
			INCLUDE( ${SFILE} )
		ENDIF()

#		MESSAGE( STATUS "IMPORT_INCLUDE_DIRS: end" )
ENDFUNCTION( IMPORT_INCLUDE_DIRS )

ENDIF()
