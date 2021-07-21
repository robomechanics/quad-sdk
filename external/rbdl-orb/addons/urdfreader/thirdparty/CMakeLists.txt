FIND_FILE(URDF_MODULE CMakeLists.txt PATHS ${CMAKE_CURRENT_SOURCE_DIR}/urdfparser NO_DEFAULT_PATH)
IF (URDF_MODULE STREQUAL URDF_MODULE-NOTFOUND)
  MESSAGE(FATAL_ERROR "The urdfreader module was not initalized correctly! Please make sure that you recursively cloned rbdl. For more information take a closer look at the README!")
ENDIF(URDF_MODULE STREQUAL URDF_MODULE-NOTFOUND)
ADD_SUBDIRECTORY(
	urdfparser
)
