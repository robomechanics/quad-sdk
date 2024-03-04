#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "alpaqa::alpaqa" for configuration "Release"
set_property(TARGET alpaqa::alpaqa APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(alpaqa::alpaqa PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libalpaqa.so.1.0.0"
  IMPORTED_SONAME_RELEASE "libalpaqa.so.1.0.0"
  )

list(APPEND _cmake_import_check_targets alpaqa::alpaqa )
list(APPEND _cmake_import_check_files_for_alpaqa::alpaqa "${_IMPORT_PREFIX}/lib64/libalpaqa.so.1.0.0" )

# Import target "alpaqa::dl-loader" for configuration "Release"
set_property(TARGET alpaqa::dl-loader APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(alpaqa::dl-loader PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libalpaqa-dl-loader.so.1.0.0"
  IMPORTED_SONAME_RELEASE "libalpaqa-dl-loader.so.1.0.0"
  )

list(APPEND _cmake_import_check_targets alpaqa::dl-loader )
list(APPEND _cmake_import_check_files_for_alpaqa::dl-loader "${_IMPORT_PREFIX}/lib64/libalpaqa-dl-loader.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
