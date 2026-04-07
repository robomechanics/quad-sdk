#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "teleop_twist_joy::teleop_twist_joy" for configuration ""
set_property(TARGET teleop_twist_joy::teleop_twist_joy APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(teleop_twist_joy::teleop_twist_joy PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libteleop_twist_joy.so"
  IMPORTED_SONAME_NOCONFIG "libteleop_twist_joy.so"
  )

list(APPEND _cmake_import_check_targets teleop_twist_joy::teleop_twist_joy )
list(APPEND _cmake_import_check_files_for_teleop_twist_joy::teleop_twist_joy "${_IMPORT_PREFIX}/lib/libteleop_twist_joy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
