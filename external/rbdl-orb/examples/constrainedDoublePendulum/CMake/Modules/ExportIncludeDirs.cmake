# File:   ExportIncludeDirs.cmake
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

IF( NOT _EXPORTINCLUDEDIRS_ )
	SET( _EXPORTINCLUDEDIRS_ TRUE )

	INCLUDE( ProblemHandling )

	FUNCTION( EXPORT_INCLUDE_DIRS TFILE DIRS )

		# Check arguments
		IF( NOT ${TFILE} MATCHES ".+" )
			FATAL_ERROR( "EXPORT_INCLUDE_DIRS called with invalid target file name.")
		ENDIF()

		# Extract dir paths from ${DIRS}
		SET( IPATHS )
		LIST( REMOVE_DUPLICATES DIRS )

		FOREACH( XXX ${DIRS} )
			GET_FILENAME_COMPONENT( XXX "${XXX}" ABSOLUTE )
				LIST( APPEND IPATHS "${XXX}" )
		ENDFOREACH( XXX )

		LIST( REMOVE_DUPLICATES IPATHS )

#		MESSAGE( STATUS  "EXPORT_INCLUDE_DIRS: exported '${IPATHS}'" )

		FILE( WRITE "${TFILE}"
"SET( ${PROJECT_NAME}_INCLUDE_DIRS \"${IPATHS}\" )
SET( ${PROJECT_NAME}_INCLUDE_DIRS \"${IPATHS}\" PARENT_SCOPE )
FOREACH( DIR \${${PROJECT_NAME}_INCLUDE_DIRS} )
	INCLUDE_DIRECTORIES( \${DIR} )
ENDFOREACH( DIR )"
)

	ENDFUNCTION( EXPORT_INCLUDE_DIRS )

ENDIF()
