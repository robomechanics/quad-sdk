

set(HIGHS_DIR "")

if (FAST_BUILD)
  if(NOT TARGET highs)
    include("${CMAKE_CURRENT_LIST_DIR}/highs-targets.cmake")
  endif()

  set(HIGHS_LIBRARIES highs)
else()
  if(NOT TARGET libhighs)
    include("${CMAKE_CURRENT_LIST_DIR}/highs-targets.cmake")
  endif()

  set(HIGHS_LIBRARIES libhighs)
endif() 

set(HIGHS_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include/highs")

set(HIGHS_FOUND TRUE)

include(CMakeFindDependencyMacro)
find_dependency(ZLIB)
