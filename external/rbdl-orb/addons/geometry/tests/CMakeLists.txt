CMAKE_MINIMUM_REQUIRED (VERSION 3.0)

SET ( RBDL_ADDON_GEOMETRY_TESTS_VERSION_MAJOR 1 )
SET ( RBDL_ADDON_GEOMETRY_TESTS_VERSION_MINOR 0 )
SET ( RBDL_ADDON_GEOMETRY_TESTS_VERSION_PATCH 0 )

SET ( RBDL_ADDON_GEOMETRY_TESTS_VERSION 
	${RBDL_ADDON_GEOMETRY_TESTS_VERSION_MAJOR}.${RBDL_ADDON_GEOMETRY_TESTS_VERSION_MINOR}.${RBDL_ADDON_GEOMETRY_TESTS_VERSION_PATCH}
)


PROJECT (RBDL_GEOMETRY_TESTS VERSION ${RBDL_ADDON_GEOMETRY_TESTS_VERSION})

# Look for catch2
FIND_PACKAGE(Catch2 REQUIRED)

SET ( GEOMETRY_TESTS_SRCS
	testSmoothSegmentedFunction.cc
	numericalTestFunctions.cc
	numericalTestFunctions.h
	../geometry.h
	../SegmentedQuinticBezierToolkit.h
	../SmoothSegmentedFunction.h
	../SegmentedQuinticBezierToolkit.cc
	../SmoothSegmentedFunction.cc
	)

INCLUDE_DIRECTORIES ( ../ )
INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/tests)
# on unix systems add /usr/local/include to include dirs
# this was added since macos homebrew installs many things there but macos
# does not automatically search their, and on linux systems this does not hurt
IF (NOT WIN32)
  INCLUDE_DIRECTORIES(/usr/local/include)
ENDIF (NOT WIN32)


SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
)

ADD_EXECUTABLE ( rbdl_geometry_tests ${GEOMETRY_TESTS_SRCS} )

SET_TARGET_PROPERTIES ( rbdl_geometry_tests PROPERTIES
	LINKER_LANGUAGE CXX
	OUTPUT_NAME geometry_tests
	)

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
	SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

TARGET_LINK_LIBRARIES ( rbdl_geometry_tests
		${UNITTEST++_LIBRARY}
		${RBDL_LIBRARY}
	)
	
OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

IF (RUN_AUTOMATIC_TESTS)
ADD_CUSTOM_COMMAND (TARGET rbdl_geometry_tests
	POST_BUILD
	COMMAND ./geometry_tests
	COMMENT "Running automated addon geometry tests..."
	)
ENDIF (RUN_AUTOMATIC_TESTS)
