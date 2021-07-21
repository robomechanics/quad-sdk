SET (RBDL_FOUND FALSE)
SET (RBDL_LUAMODEL_FOUND FALSE)
SET (RBDL_URDFREADER_FOUND FALSE)
SET (RBDL_GEOMETRY_FOUND FALSE)
SET (RBDL_MUSCLE_FOUND FALSE)


UNSET( RBDL_INCLUDE_DIR              CACHE)   
UNSET( RBDL_LIBRARY                  CACHE)    
UNSET( RBDL_LUAMODEL_INCLUDE_DIR     CACHE)   
UNSET( RBDL_LUAMODEL_LIBRARY         CACHE)   
UNSET( RBDL_URDFREADER_INCLUDE_DIR   CACHE)    
UNSET( RBDL_URDFREADER_LIBRARY       CACHE)     
UNSET( RBDL_MUSCLE_INCLUDE_DIR       CACHE)      
UNSET( RBDL_MUSCLE_LIBRARY           CACHE)    
UNSET( RBDL_GEOMETRY_INCLUDE_DIR     CACHE)   
UNSET( RBDL_GEOMETRY_LIBRARY         CACHE)   

IF(CUSTOM_RBDL_PATH)

  FIND_PATH (RBDL_INCLUDE_DIR rbdl/rbdl.h
    PATHS 
    ${CUSTOM_RBDL_PATH}/include 
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY (RBDL_LIBRARY rbdl
    PATHS
    ${CUSTOM_RBDL_PATH}/lib
    NO_DEFAULT_PATH
    )

  FIND_PATH (RBDL_LUAMODEL_INCLUDE_DIR rbdl/addons/luamodel/luamodel.h
    PATHS
    ${CUSTOM_RBDL_PATH}/include
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY (RBDL_LUAMODEL_LIBRARY rbdl_luamodel
    PATHS
    ${CUSTOM_RBDL_PATH}/lib
    NO_DEFAULT_PATH
    )

  FIND_PATH (RBDL_URDFREADER_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
    PATHS
    ${CUSTOM_RBDL_PATH}/include
    NO_DEFAULT_PATH  
    )


  FIND_LIBRARY (RBDL_URDFREADER_LIBRARY rbdl_urdfreader
    PATHS
    ${CUSTOM_RBDL_PATH}/lib
    NO_DEFAULT_PATH  
    )


  FIND_PATH (RBDL_MUSCLE_INCLUDE_DIR rbdl/addons/muscle/muscle.h
    PATHS
    ${CUSTOM_RBDL_PATH}/include
    NO_DEFAULT_PATH
    )


  FIND_LIBRARY (RBDL_MUSCLE_LIBRARY rbdl_muscle  
    PATHS
    ${CUSTOM_RBDL_PATH}/lib
    NO_DEFAULT_PATH
    )

  FIND_PATH (RBDL_GEOMETRY_INCLUDE_DIR rbdl/addons/geometry/geometry.h
    PATHS
    ${CUSTOM_RBDL_PATH}/include
    NO_DEFAULT_PATH  
    )

  FIND_LIBRARY (RBDL_GEOMETRY_LIBRARY rbdl_geometry 
    PATHS
    ${CUSTOM_RBDL_PATH}/lib
    NO_DEFAULT_PATH  
    )

ELSE(CUSTOM_RBDL_PATH)
  FIND_PATH (RBDL_INCLUDE_DIR rbdl/rbdl.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
    )

  FIND_LIBRARY (RBDL_LIBRARY rbdl
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}/lib
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )

  FIND_PATH (RBDL_LUAMODEL_INCLUDE_DIR rbdl/addons/luamodel/luamodel.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
    )

  FIND_LIBRARY (RBDL_LUAMODEL_LIBRARY rbdl_luamodel
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )

  FIND_PATH (RBDL_URDFREADER_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
    )

  FIND_LIBRARY (RBDL_URDFREADER_LIBRARY rbdl_urdfreader
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )


  FIND_PATH (RBDL_MUSCLE_INCLUDE_DIR rbdl/addons/muscle/muscle.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
    )

  FIND_LIBRARY (RBDL_MUSCLE_LIBRARY rbdl_muscle  
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )

  FIND_PATH (RBDL_GEOMETRY_INCLUDE_DIR rbdl/addons/geometry/geometry.h
    HINTS
    $ENV{HOME}/local/include
    $ENV{RBDL_PATH}/src
    $ENV{RBDL_PATH}/include
    $ENV{RBDL_INCLUDE_PATH}
    /usr/local/include
    /usr/include
    )

  FIND_LIBRARY (RBDL_GEOMETRY_LIBRARY rbdl_geometry  
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{RBDL_PATH}
    $ENV{RBDL_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )  
ENDIF(CUSTOM_RBDL_PATH)

IF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)
  SET (RBDL_FOUND TRUE)
ELSE(RBDL_INCLUDE_DIR AND RBDL_LIBRARY)
  IF(RBDL_FIND_REQUIRED)
    MESSAGE (SEND_ERROR " Could not find RBDL.")
    MESSAGE (SEND_ERROR " Try setting CUSTOM_RBDL_PATH in FindRBDL.cmake force CMake to use the desired directory.")
  ELSE(RBDL_FIND_REQUIRED)
    MESSAGE (STATUS " Could not find RBDL.")
    MESSAGE (STATUS " Try setting CUSTOM_RBDL_PATH in FindRBDL.cmake force CMake to use the desired directory.")
  ENDIF(RBDL_FIND_REQUIRED)
ENDIF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)


IF (RBDL_LUAMODEL_LIBRARY AND RBDL_LUAMODEL_INCLUDE_DIR)
  SET (RBDL_LUAMODEL_FOUND TRUE)
ENDIF (RBDL_LUAMODEL_LIBRARY AND RBDL_LUAMODEL_INCLUDE_DIR)

IF (RBDL_URDFREADER_LIBRARY AND RBDL_URDFREADER_INCLUDE_DIR)
  SET (RBDL_URDFREADER_FOUND TRUE)
ENDIF (RBDL_URDFREADER_LIBRARY AND RBDL_URDFREADER_INCLUDE_DIR)

IF (RBDL_MUSCLE_LIBRARY AND RBDL_MUSCLE_INCLUDE_DIR)
  SET (RBDL_MUSCLE_FOUND TRUE)
ENDIF (RBDL_MUSCLE_LIBRARY AND RBDL_MUSCLE_INCLUDE_DIR)

IF (RBDL_GEOMETRY_LIBRARY AND RBDL_GEOMETRY_INCLUDE_DIR)
  SET (RBDL_GEOMETRY_FOUND TRUE)
ENDIF (RBDL_GEOMETRY_LIBRARY AND RBDL_GEOMETRY_INCLUDE_DIR)


IF (RBDL_FOUND)
   IF (NOT RBDL_FIND_QUIETLY)
      MESSAGE(STATUS "Found RBDL: ${RBDL_LIBRARY}")
   ENDIF (NOT RBDL_FIND_QUIETLY)

   foreach ( COMPONENT ${RBDL_FIND_COMPONENTS} )
     IF (RBDL_${COMPONENT}_FOUND)
       IF (NOT RBDL_FIND_QUIETLY)
         MESSAGE(STATUS "Found RBDL ${COMPONENT}: ${RBDL_${COMPONENT}_LIBRARY}")
       ENDIF (NOT RBDL_FIND_QUIETLY)
     ELSE (RBDL_${COMPONENT}_FOUND)
       MESSAGE(ERROR " Could not find RBDL ${COMPONENT}")
     ENDIF (RBDL_${COMPONENT}_FOUND)
   endforeach ( COMPONENT )

ENDIF (RBDL_FOUND)

MARK_AS_ADVANCED (
  RBDL_INCLUDE_DIR
  RBDL_LIBRARY
  RBDL_LUAMODEL_INCLUDE_DIR
  RBDL_LUAMODEL_LIBRARY
  RBDL_URDFREADER_INCLUDE_DIR
  RBDL_URDFREADER_LIBRARY
  RBDL_MUSCLE_INCLUDE_DIR
  RBDL_MUSCLE_LIBRARY
  RBDL_GEOMETRY_INCLUDE_DIR
  RBDL_GEOMETRY_LIBRARY
  )
