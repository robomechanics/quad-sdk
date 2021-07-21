#     3a. This file is going to look for rbdl, rbdl-luamodel, rbdl-urdfreader,
#         rbdl-geometry, and rbdl-muscle. Here we make a bunch of boolean flags
#         to indicate if these resources have been found and initialize them 
#         to false.

SET (RBDL_FOUND FALSE)
SET (RBDL_LUAMODEL_FOUND FALSE)
SET (RBDL_URDFREADER_FOUND FALSE)
SET (RBDL_GEOMETRY_FOUND FALSE)
SET (RBDL_MUSCLE_FOUND FALSE)

#     3b. All of the variables that this FindRBDL.cmake file will populate, 
#         if these resources exist, are listed here.

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


#     3c. This script has two different modes:
#
#        If there is a CUSTOM_RBDL_PATH: then this path is used to search 
#        for Rbdl
#
#        If there is no CUSTOM_RBDL_PATH, then a bunch of typical install
#        locations are used. Note that at the present time these typical install
#        include paths that will only work on linux

IF(CUSTOM_RBDL_PATH)


#     3d. The validity of each path is checked by using the FIND_PATH 
#         command to look for a specific file. In the case of the RBDL_INCLUDE_DIR
#         it is the correct path if you can find rbdl/rbdl.h from it. 
#         If that is true then RBDL_INCLUDE_DIR is assigned 
#         ${CUSTOM_RBDL_PATH}/include (remember the $ converts the variable 
#         to its string representation)

  FIND_PATH (RBDL_INCLUDE_DIR rbdl/rbdl.h
    PATHS 
    ${CUSTOM_RBDL_PATH}/include 
    NO_DEFAULT_PATH
    )


#     3e. Similarly the validity of a path to a library is checked by looking
#         to see if a specific library exists using the FIND_LIBRARY command.
#         Note that you do not need to put the file type on the end of the 
#         library name, nor a prefix of 'lib': CMake will do this for you 
#         in a way that is cross-platform.

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


#     3f. If there is no CUSTOM_RBDL_PATH given then FIND_PATH and FIND_LIBRARY
#         commands are used but with substantial HINTS, or places to look

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


#     3g. If we've gotten to this point then either all include directories 
#         and libraries have been found, some have been found, or none have 
#         been found. All of the code below is going through what the user
#         asked for, seeing if it was found, and if not issuing an error.


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


#     3h. Here all of the specific paths and libraries are marked as advanced
#         which means that they will not appear in the CMake gui unless the 
#         user toggles to the advanced mode.                

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
