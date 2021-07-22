CMAKE_MINIMUM_REQUIRED (VERSION 3.0)

# Look for catch2
FIND_PACKAGE(Catch2 REQUIRED)
# on unix systems add /usr/local/include to include dirs
# this was added since macos homebrew installs many things there but macos
# does not automatically search their, and on linux systems this does not hurt
IF (NOT WIN32)
  INCLUDE_DIRECTORIES(/usr/local/include)
ENDIF (NOT WIN32)

SET ( TESTS_SRCS
  main.cc
  MathTests.cc
  ErrorTests.cc
  SpatialAlgebraTests.cc
  MultiDofTests.cc
  KinematicsTests.cc
  BodyTests.cc
  ModelTests.cc
  FloatingBaseTests.cc
  CalcVelocitiesTests.cc
  CalcAccelerationsTests.cc
  DynamicsTests.cc
  InverseDynamicsTests.cc
  CompositeRigidBodyTests.cc
  ImpulsesTests.cc
  TwolegModelTests.cc
  ContactsTests.cc
  UtilsTests.cc
  SparseFactorizationTests.cc
  CustomJointSingleBodyTests.cc
  CustomJointMultiBodyTests.cc
  CustomConstraintsTests.cc
  InverseKinematicsTests.cc
  LoopConstraintsTests.cc
  ScrewJointTests.cc
  ForwardDynamicsConstraintsExternalForces.cc
  InverseDynamicsWithConstraintsTests.cc  
  )

INCLUDE_DIRECTORIES ( ${CMAKE_SOURCE_DIR}/include/ )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
  OUTPUT_NAME rbdl_tests
  )

ADD_EXECUTABLE ( rbdl_tests ${TESTS_SRCS} )

SET_TARGET_PROPERTIES ( rbdl_tests PROPERTIES
  LINKER_LANGUAGE CXX
  )

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
  SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

TARGET_LINK_LIBRARIES ( rbdl_tests
  Catch2::Catch2
  ${RBDL_LIBRARY}
  )

OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

IF (RUN_AUTOMATIC_TESTS)
  ADD_CUSTOM_COMMAND (TARGET rbdl_tests
    POST_BUILD
    COMMAND ./rbdl_tests
    COMMENT "Running automated tests..."
    )
ENDIF (RUN_AUTOMATIC_TESTS)
