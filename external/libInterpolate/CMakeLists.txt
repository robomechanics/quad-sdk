cmake_minimum_required( VERSION 3.10 )

if( TARGET libInterpolate )
  return()
endif( TARGET libInterpolate )

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR} )
  set(STANDALONE ON)
endif( )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(libInterpolate)
include(macro-git_version)
GIT_VERSION(libInterpolate)
message( STATUS "libInterpolate version: ${${PROJECT_NAME}_VERSION}")

# need to run configure_file twice
configure_file( "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/${PROJECT_NAME}_version.h.in"
                "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_version.h")
configure_file( "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_version.h"
                "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${PROJECT_NAME}_version.h")


set( cxx_needed_features
     cxx_auto_type
   )

find_package( Boost REQUIRED )
find_package( Eigen3 3.3.1 REQUIRED CONFIG ) # v3.3.1 adds support for cmake targets

string( REGEX REPLACE "^lib" "" LIB_NAME ${PROJECT_NAME} )
add_library( ${LIB_NAME} INTERFACE )
add_library( ${PROJECT_NAME}::${LIB_NAME} ALIAS ${LIB_NAME} )
target_include_directories( ${LIB_NAME} INTERFACE
  $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${${PROJECT_NAME}_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions( ${LIB_NAME}
INTERFACE
  $<$<BOOL:${MSVC}>:_USE_MATH_DEFINES>
)
target_sources(
  ${LIB_NAME}
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interp.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolate.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/AnyInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_2D/InterpolatorBase.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_2D/ThinPlateSplineInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_2D/BilinearInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_2D/AnyInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_2D/BicubicInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_1D/InterpolatorBase.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_1D/CubicSplineInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_1D/AnyInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_1D/MonotonicInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Interpolators/_1D/LinearInterpolator.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Utils/Indexing.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Utils/ReadFunction.hpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/Utils/Concepts.hpp>
)

target_link_libraries(${LIB_NAME} INTERFACE Boost::boost Eigen3::Eigen)

install( TARGETS ${LIB_NAME} EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY src/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION cmake
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
"include(CMakeFindDependencyMacro)
find_dependency(Boost)
find_dependency(Eigen3 3.3.1 CONFIG)
include(\${CMAKE_CURRENT_LIST_DIR}/${PROJECT_NAME}Targets.cmake)
"
  )
include(CMakePackageConfigHelpers)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION cmake
  )

INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A C++ library for numerical interpolation.")
SET(CPACK_PACKAGE_VENDOR "C.D. Clark III")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.md")
if( ${PROJECT_NAME}_VERSION_MAJOR)
  SET(CPACK_PACKAGE_VERSION_MAJOR ${${PROJECT_NAME}_VERSION_MAJOR})
endif()
if( ${PROJECT_NAME}_VERSION_MINOR)
  SET(CPACK_PACKAGE_VERSION_MINOR ${${PROJECT_NAME}_VERSION_MINOR})
endif()
if( ${PROJECT_NAME}_VERSION_PATCH)
  SET(CPACK_PACKAGE_VERSION_PATCH ${${PROJECT_NAME}_VERSION_PATCH})
endif()
INCLUDE(CPack)







# vvvvDEPRECATEDvvvv

# the code below is provided for backwards compatibility
# for when the library was named libInterp instead of libInterpolate

add_library( Interp INTERFACE )
add_library( libInterp::Interp ALIAS Interp )
target_include_directories( Interp INTERFACE
  $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${${PROJECT_NAME}_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/src/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions( Interp
INTERFACE
  $<$<BOOL:${MSVC}>:_USE_MATH_DEFINES>
)
target_link_libraries(Interp INTERFACE Boost::boost Eigen3::Eigen)


install( TARGETS Interp EXPORT libInterpTargets)
install(EXPORT libInterpTargets
  FILE libInterpTargets.cmake
  NAMESPACE libInterp::
  DESTINATION cmake
)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/libInterpConfig.cmake
"message(WARNING \"libInterp has been renamed libInterpolate. Please use find_package(libInterpolate) and the libInterpolate::Interpolate target.\")
include(CMakeFindDependencyMacro)
find_dependency(Boost)
find_dependency(Eigen3 3.3.1 CONFIG)
include(\${CMAKE_CURRENT_LIST_DIR}/libInterpTargets.cmake)
"
  )
include(CMakePackageConfigHelpers)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/libInterpConfig.cmake
  DESTINATION cmake
  )
