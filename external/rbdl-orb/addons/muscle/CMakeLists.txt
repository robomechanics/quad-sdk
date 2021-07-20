CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

#CMAKE_POLICY(SET CMP0048 NEW)
#CMAKE_POLICY(SET CMP0040 NEW)
 
SET ( RBDL_ADDON_MUSCLE_VERSION_MAJOR 2 )
SET ( RBDL_ADDON_MUSCLE_VERSION_MINOR 0 )
SET ( RBDL_ADDON_MUSCLE_VERSION_PATCH 0 )

SET ( RBDL_ADDON_MUSCLE_VERSION 
  ${RBDL_ADDON_MUSCLE_VERSION_MAJOR}.${RBDL_ADDON_MUSCLE_VERSION_MINOR}.${RBDL_ADDON_MUSCLE_VERSION_PATCH}
)



PROJECT (RBDL_ADDON_MUSCLE VERSION ${RBDL_ADDON_MUSCLE_VERSION})
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )
SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
)

IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
  FIND_PACKAGE (IPOPT REQUIRED)
  INCLUDE_DIRECTORIES ( 
    ${CMAKE_CURRENT_BINARY_DIR}/include/rbdl
    ${IPOPT_INCLUDE_DIR}
  )
  SET(MUSCLE_SOURCES 
    muscle.h
    Millard2016TorqueMuscle.h
    Millard2016TorqueMuscle.cc
    TorqueMuscleFittingToolkit.h
    TorqueMuscleFittingToolkit.cc
    MuscleFunctionFactory.h 
    MuscleFunctionFactory.cc
    TorqueMuscleFunctionFactory.h
    TorqueMuscleFunctionFactory.cc
    csvtools.h
    csvtools.cc
  )

  SET(MUSCLE_HEADERS  
    muscle.h
    Millard2016TorqueMuscle.h
    TorqueMuscleFittingToolkit.h    
    MuscleFunctionFactory.h 
    TorqueMuscleFunctionFactory.h
    csvtools.h  
  )


ELSE(RBDL_BUILD_ADDON_MUSCLE_FITTING)
  INCLUDE_DIRECTORIES ( 
    ${CMAKE_CURRENT_BINARY_DIR}/include/rbdl
  )

  SET(MUSCLE_SOURCES 
    muscle.h
    Millard2016TorqueMuscle.h
    Millard2016TorqueMuscle.cc
    MuscleFunctionFactory.h 
    MuscleFunctionFactory.cc
    TorqueMuscleFunctionFactory.h
    TorqueMuscleFunctionFactory.cc
    csvtools.h
    csvtools.cc
  )

  SET(MUSCLE_HEADERS  
    muscle.h
    Millard2016TorqueMuscle.h
    MuscleFunctionFactory.h 
    TorqueMuscleFunctionFactory.h
    csvtools.h  
  )

ENDIF(RBDL_BUILD_ADDON_MUSCLE_FITTING)
  


IF (RBDL_BUILD_STATIC)

  ADD_LIBRARY ( rbdl_muscle-static STATIC ${MUSCLE_SOURCES} )
  SET_TARGET_PROPERTIES ( rbdl_muscle-static PROPERTIES PREFIX "lib")
  SET_TARGET_PROPERTIES ( rbdl_muscle-static PROPERTIES OUTPUT_NAME "rbdl_muscle")

  IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
    TARGET_LINK_LIBRARIES (rbdl_muscle-static
      rbdl_geometry-static
      rbdl-static
      ${IPOPT_LIBRARY})
  ELSE(RBDL_BUILD_ADDON_MUSCLE_FITTING)
    TARGET_LINK_LIBRARIES (rbdl_muscle-static
      rbdl_geometry-static
      rbdl-static)
  ENDIF(RBDL_BUILD_ADDON_MUSCLE_FITTING)

  INSTALL (TARGETS rbdl_muscle-static  
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
ELSE (RBDL_BUILD_STATIC)

  ADD_LIBRARY ( rbdl_muscle SHARED ${MUSCLE_SOURCES} )
  SET_TARGET_PROPERTIES ( rbdl_muscle PROPERTIES
    VERSION ${RBDL_VERSION}
    SOVERSION ${RBDL_SO_VERSION}
  )

  IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
    TARGET_LINK_LIBRARIES (
      rbdl_muscle
      rbdl_geometry
      rbdl
      ${IPOPT_LIBRARY}
    )
  ELSE (RBDL_BUILD_ADDON_MUSCLE_FITTING)
    TARGET_LINK_LIBRARIES (
      rbdl_muscle
      rbdl_geometry
      rbdl
    )
  ENDIF(RBDL_BUILD_ADDON_MUSCLE_FITTING)

  INSTALL (TARGETS rbdl_muscle
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
  
ENDIF (RBDL_BUILD_STATIC)

FILE ( GLOB headers 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

INSTALL ( FILES ${MUSCLE_HEADERS} 
  DESTINATION 
  ${CMAKE_INSTALL_INCLUDEDIR}/rbdl/addons/muscle
)
