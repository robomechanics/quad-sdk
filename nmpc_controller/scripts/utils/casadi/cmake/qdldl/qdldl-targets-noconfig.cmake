#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "qdldl::qdldlstatic" for configuration ""
set_property(TARGET qdldl::qdldlstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qdldl::qdldlstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libqdldl.a"
  )

list(APPEND _cmake_import_check_targets qdldl::qdldlstatic )
list(APPEND _cmake_import_check_files_for_qdldl::qdldlstatic "${_IMPORT_PREFIX}/lib/libqdldl.a" )

# Import target "qdldl::qdldl" for configuration ""
set_property(TARGET qdldl::qdldl APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(qdldl::qdldl PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libqdldl.so"
  IMPORTED_SONAME_NOCONFIG "libqdldl.so"
  )

list(APPEND _cmake_import_check_targets qdldl::qdldl )
list(APPEND _cmake_import_check_files_for_qdldl::qdldl "${_IMPORT_PREFIX}/lib/libqdldl.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
