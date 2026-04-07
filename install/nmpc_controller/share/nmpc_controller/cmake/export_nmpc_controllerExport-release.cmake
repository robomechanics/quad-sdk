#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "nmpc_controller::nmpc_controller" for configuration "Release"
set_property(TARGET nmpc_controller::nmpc_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nmpc_controller::nmpc_controller PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libnmpc_controller.a"
  )

list(APPEND _cmake_import_check_targets nmpc_controller::nmpc_controller )
list(APPEND _cmake_import_check_files_for_nmpc_controller::nmpc_controller "${_IMPORT_PREFIX}/lib/libnmpc_controller.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
