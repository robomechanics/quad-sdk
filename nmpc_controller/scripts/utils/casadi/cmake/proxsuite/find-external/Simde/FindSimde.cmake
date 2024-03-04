#
# Copyright (c) 2022 INRIA
#

find_path(Simde_INCLUDE_DIR simde/simde-math.h)

if(NOT SIMDE_HINT_FAILURE)
  set(SIMDE_HINT_FAILURE None)
endif()

if(${CMAKE_VERSION} VERSION_LESS "3.16.0")
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    Simde
    FOUND_VAR Simde_FOUND
    REQUIRED_VARS Simde_INCLUDE_DIR)
else()
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    Simde
    FOUND_VAR Simde_FOUND
    REQUIRED_VARS Simde_INCLUDE_DIR REASON_FAILURE_MESSAGE
                  ${SIMDE_HINT_FAILURE})
endif()

if(Simde_FOUND)
  add_library(simde INTERFACE IMPORTED)
  set_target_properties(simde PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                         "${Simde_INCLUDE_DIR}")
endif()

mark_as_advanced(SIMDE_INCLUDE_DIR)

include(FeatureSummary)
set_package_properties(
  Simde PROPERTIES
  DESCRIPTION
    "Implementations of SIMD instruction sets for systems which don't natively support them."
  URL "https://github.com/simd-everywhere/simde")
