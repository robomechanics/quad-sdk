#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sleqp::sleqp" for configuration "Release"
set_property(TARGET sleqp::sleqp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(sleqp::sleqp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "trlib::trlib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libsleqp.so.1.0.1"
  IMPORTED_SONAME_RELEASE "libsleqp.so.1.0.1"
  )

list(APPEND _cmake_import_check_targets sleqp::sleqp )
list(APPEND _cmake_import_check_files_for_sleqp::sleqp "${_IMPORT_PREFIX}/lib64/libsleqp.so.1.0.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
