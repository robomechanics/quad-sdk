# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_force_applicator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED force_applicator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(force_applicator_FOUND FALSE)
  elseif(NOT force_applicator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(force_applicator_FOUND FALSE)
  endif()
  return()
endif()
set(_force_applicator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT force_applicator_FIND_QUIETLY)
  message(STATUS "Found force_applicator: 0.0.0 (${force_applicator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'force_applicator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT force_applicator_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(force_applicator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${force_applicator_DIR}/${_extra}")
endforeach()
