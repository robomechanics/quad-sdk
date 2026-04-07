# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_global_body_planner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED global_body_planner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(global_body_planner_FOUND FALSE)
  elseif(NOT global_body_planner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(global_body_planner_FOUND FALSE)
  endif()
  return()
endif()
set(_global_body_planner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT global_body_planner_FIND_QUIETLY)
  message(STATUS "Found global_body_planner: 0.0.0 (${global_body_planner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'global_body_planner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT global_body_planner_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(global_body_planner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${global_body_planner_DIR}/${_extra}")
endforeach()
