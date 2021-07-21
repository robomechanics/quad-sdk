IF (RBDL_USE_PYTHON_2)
	FIND_PROGRAM ( PYTHON "python2" )
	SET (Python_ADDITIONAL_VERSIONS 2.7)
	INCLUDE ( FindNumPy2 )
	SET( PYTHON_BIN python2)
ELSE(RBDL_USE_PYTHON_2)
	FIND_PROGRAM ( PYTHON "python3" )
	SET (Python_ADDITIONAL_VERSIONS 3.7)
	INCLUDE ( FindNumPy3 )
	SET( PYTHON_BIN python3)
ENDIF (RBDL_USE_PYTHON_2)

INCLUDE ( UseCython )

FILE( COPY "rbdl.pxd" "rbdlmuscle.pyx" "crbdl.pxd" "crbdlmuscle.pxd" "ICustomJoint.cpp" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
FILE( COPY "test_wrapper.py" "test_rbdlmuscle.py" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

#FILE( COPY "rbdl.pyx" "rbdl.pxd" "crbdl.pxd" "ICustomJoint.cpp" "rbdlmuscle.pyx" "crbdlmuscle.pxd" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

CONFIGURE_FILE (
  ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/setup.py
  )

# Process the rbdl-wrapper.pyx to generate rbdl.pyx BEFORE build time
# else cython_add_module will complain on configuration
EXECUTE_PROCESS(
	COMMAND ${PYTHON}
	${CMAKE_CURRENT_SOURCE_DIR}/wrappergen.py
	${CMAKE_CURRENT_SOURCE_DIR}/rbdl-wrapper.pyx
	${CMAKE_CURRENT_BINARY_DIR}/rbdl.pyx
)


# Enable C++11 (or C++0x for older compilers)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# If the pyx file is a C++ file, we should specify that here.
set_source_files_properties( ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pyx PROPERTIES CYTHON_IS_CXX TRUE )

# Multi-file cython modules do not appear to be working at the moment.
# NOTE there is some directory property problem in UseCython when build
#      from CMAKE_CURRENT_BINARY_DIR
cython_add_module( rbdl-python ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pyx ICustomJoint.cpp )

#SET_TARGET_PROPERTIES ( rbdl-python PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES ( rbdl-python PROPERTIES OUTPUT_NAME "rbdl")

IF (RBDL_BUILD_ADDON_MUSCLE)
  set_source_files_properties( ${CMAKE_CURRENT_BINARY_DIR}/rbdlmuscle.pyx PROPERTIES CYTHON_IS_CXX TRUE ) 
  cython_add_module( rbdlmuscle-python ${CMAKE_CURRENT_BINARY_DIR}/rbdlmuscle.pyx )
  SET_TARGET_PROPERTIES ( rbdlmuscle-python PROPERTIES OUTPUT_NAME "rbdlmuscle")
ENDIF()

IF (RBDL_USE_PYTHON_2)
	INCLUDE_DIRECTORIES (
	  ${PROJECT_SOURCE_DIR}/include
	  ${PROJECT_SOURCE_DIR}/python
	  ${PROJECT_SOURCE_DIR}
	  ${PYTHON_NUMPY_2_INCLUDE_DIR}
	)
ELSE(RBDL_USE_PYTHON_2)
	INCLUDE_DIRECTORIES (
	  ${PROJECT_SOURCE_DIR}/include
	  ${PROJECT_SOURCE_DIR}/python
	  ${PROJECT_SOURCE_DIR}
	  ${PYTHON_NUMPY_3_INCLUDE_DIR}
	)
ENDIF (RBDL_USE_PYTHON_2)

IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
  FIND_PACKAGE (IPOPT REQUIRED)
  INCLUDE_DIRECTORIES (
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/python
  ${PROJECT_SOURCE_DIR}
  ${IPOPT_INCLUDE_DIR}
  )
ENDIF()


TARGET_LINK_LIBRARIES (rbdl-python rbdl )

IF (RBDL_BUILD_ADDON_LUAMODEL) 
  TARGET_LINK_LIBRARIES (rbdl-python rbdl_luamodel )
ENDIF()

IF (RBDL_BUILD_ADDON_URDFREADER) 
  TARGET_LINK_LIBRARIES (rbdl-python rbdl_urdfreader )
ENDIF()

IF (RBDL_BUILD_ADDON_MUSCLE) 
  TARGET_LINK_LIBRARIES (rbdlmuscle-python rbdl rbdl_geometry rbdl_muscle )
  IF (RBDL_BUILD_ADDON_MUSCLE_FITTING) 
    TARGET_LINK_LIBRARIES (rbdlmuscle-python ${IPOPT_LIBRARY} )
  ENDIF()
  IF (RBDL_BUILD_ADDON_LUAMODEL) 
    TARGET_LINK_LIBRARIES (rbdlmuscle-python rbdl_luamodel )
  ENDIF()
  IF (RBDL_BUILD_ADDON_URDFREADER) 
    TARGET_LINK_LIBRARIES (rbdlmuscle-python rbdl_urdfreader )
  ENDIF()
ENDIF()

#INSTALL ( CODE "EXECUTE_PROCESS(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PYTHON_BIN} setup.py install)")
