CMAKE_MINIMUM_REQUIRED (VERSION 3.0)

#CMAKE_POLICY(SET CMP0048 NEW)
#CMAKE_POLICY(SET CMP0040 NEW)
 
SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION_MAJOR 2 )
SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION_MINOR 0 )
SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION_PATCH 0 )

SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION 
  ${RBDL_ADDON_MUSCLE_TESTS_VERSION_MAJOR}.${RBDL_ADDON_MUSCLE_TESTS_VERSION_MINOR}.${RBDL_ADDON_MUSCLE_TESTS_VERSION_PATCH}
)


PROJECT (RBDL_MUSCLE_TESTS VERSION ${RBDL_ADDON_MUSCLE_TESTS_VERSION})

# Look for catch2
FIND_PACKAGE(Catch2 REQUIRED)
# on unix systems add /usr/local/include to include dirs
# this was added since macos homebrew installs many things there but macos
# does not automatically search their, and on linux systems this does not hurt
IF (NOT WIN32)
  INCLUDE_DIRECTORIES(/usr/local/include)
ENDIF (NOT WIN32)

INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/tests)

IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
  INCLUDE_DIRECTORIES (${IPOPT_INCLUDE_DIR})
ENDIF (RBDL_BUILD_ADDON_MUSCLE_FITTING)

IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
  SET ( MUSCLE_TESTS_SRCS
    testMillard2016TorqueMuscle.cc
    testMuscleFunctionFactory.cc
    testTorqueMuscleFunctionFactory.cc
    ../muscle.h
    ../MuscleFunctionFactory.h
    ../MuscleFunctionFactory.cc
    ../TorqueMuscleFunctionFactory.h
    ../TorqueMuscleFunctionFactory.cc  
    ../Millard2016TorqueMuscle.h
    ../Millard2016TorqueMuscle.cc
    ../TorqueMuscleFittingToolkit.cc
    ../TorqueMuscleFittingToolkit.h    
    ../csvtools.h
    ../csvtools.cc
    ../../geometry/geometry.h
    ../../geometry/SegmentedQuinticBezierToolkit.h
    ../../geometry/SmoothSegmentedFunction.h
    ../../geometry/SegmentedQuinticBezierToolkit.cc
    ../../geometry/SmoothSegmentedFunction.cc
    ../../geometry/tests/numericalTestFunctions.cc
    ../../geometry/tests/numericalTestFunctions.h
  )

ELSE (RBDL_BUILD_ADDON_MUSCLE_FITTING)

  SET ( MUSCLE_TESTS_SRCS
    testMillard2016TorqueMuscle.cc
    testMuscleFunctionFactory.cc
    testTorqueMuscleFunctionFactory.cc
    ../muscle.h
    ../MuscleFunctionFactory.h
    ../MuscleFunctionFactory.cc
    ../TorqueMuscleFunctionFactory.h
      ../TorqueMuscleFunctionFactory.cc  
    ../Millard2016TorqueMuscle.h
    ../Millard2016TorqueMuscle.cc
    ../csvtools.h
    ../csvtools.cc
    ../../geometry/geometry.h
    ../../geometry/SegmentedQuinticBezierToolkit.h
    ../../geometry/SmoothSegmentedFunction.h
    ../../geometry/SegmentedQuinticBezierToolkit.cc
    ../../geometry/SmoothSegmentedFunction.cc
    ../../geometry/tests/numericalTestFunctions.cc
    ../../geometry/tests/numericalTestFunctions.h
  )


ENDIF(RBDL_BUILD_ADDON_MUSCLE_FITTING)




SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
)

ADD_EXECUTABLE ( rbdl_muscle_tests ${MUSCLE_TESTS_SRCS} )

SET_TARGET_PROPERTIES ( rbdl_muscle_tests PROPERTIES
  LINKER_LANGUAGE CXX
  OUTPUT_NAME muscle_tests
  )

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
  SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
TARGET_LINK_LIBRARIES ( rbdl_muscle_tests
  ${RBDL_LIBRARY}
  ${IPOPT_LIBRARY}
  )

ELSE (RBDL_BUILD_ADDON_MUSCLE_FITTING)
TARGET_LINK_LIBRARIES ( rbdl_muscle_tests
  ${RBDL_LIBRARY}
)
ENDIF (RBDL_BUILD_ADDON_MUSCLE_FITTING)  
OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

IF (RUN_AUTOMATIC_TESTS)
ADD_CUSTOM_COMMAND (TARGET rbdl_muscle_tests
  POST_BUILD
  COMMAND ./muscle_tests
  COMMENT "Running automated addon muscle tests..."
  )
ENDIF (RUN_AUTOMATIC_TESTS)
