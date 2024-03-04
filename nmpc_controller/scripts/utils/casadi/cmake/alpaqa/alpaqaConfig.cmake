
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

####################################################################################

include(CMakeFindDependencyMacro)
find_dependency(Eigen3 REQUIRED CONFIG)
if (OFF)
    find_package(casadi CONFIG)
endif()

set(ALPAQA_WITH_OCP ON CACHE INTERNAL "")

include("${CMAKE_CURRENT_LIST_DIR}/alpaqaTargets.cmake")

if(NOT _alpaqa_PRINTED)
    get_target_property(ALPAQA_CONFIG alpaqa::alpaqa IMPORTED_CONFIGURATIONS)
    message(STATUS "Found alpaqa 1.0.0: ${CMAKE_CURRENT_LIST_DIR} (available configs: ${ALPAQA_CONFIG})")
    set(_alpaqa_PRINTED On CACHE INTERNAL "")
endif()
