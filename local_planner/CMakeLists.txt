cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
  
## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pybind11_catkin
  spirit_msgs
  spirit_utils
  grid_map_core
  grid_map_ros
  eigen_conversions
  nmpc_controller
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PythonLibs 2.7)

set(QPOASES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/qpOASES")
set(QPOASES_INCLUDE_DIRS "${QPOASES_DIR}/include")
set(QPOASES_LIBRARIES "${QPOASES_DIR}/build/libs")
set(FADE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/fade")
set(FADE25D_INCLUDE_DIRS "${FADE_DIR}/include_fade25d")
set(FADE25D_LIBRARIES "${FADE_DIR}/libs")
find_library(QPOASES NAMES qpOASES PATHS ${QPOASES_LIBRARIES} REQUIRED)
find_library(Fade25d NAMES fade25d PATHS ${FADE25D_LIBRARIES} REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs spirit_msgs spirit_utils grid_map_core grid_map_ros eigen_conversions nmpc_controller pybind11_catkin
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#message(STATUS ${OSQP_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${QPOASES_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${FADE25D_INCLUDE_DIRS}
  /usr/local/include/coin-or
)

link_directories(/usr/local/lib)

add_library(quadruped_mpc
   src/quadruped_mpc.cpp
)
target_link_libraries(quadruped_mpc OsqpEigen::OsqpEigen )

add_library(mpc_controller
	src/mpc_controller.cpp
)
target_link_libraries(mpc_controller quadruped_mpc)
add_dependencies(mpc_controller ${mpc_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library for control
add_library(local_planner
  src/local_planner.cpp
  src/local_footstep_planner.cpp
)
target_link_libraries(local_planner quadruped_mpc)
# add_dependencies(local_planner ${local_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(local_planner_node src/local_planner_node.cpp)
add_executable(mpc_controller_node src/mpc_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(local_planner_node
  local_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(mpc_controller_node 
  mpc_controller
  ${catkin_LIBRARIES}
)

pybind11_add_module(local_planner_python MODULE src/local_planner.cpp)

target_link_libraries(local_planner_python PRIVATE local_planner quadruped_mpc ${catkin_LIBRARIES})
catkin_install_python(
  PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../gym_scripts/python/examples/hello_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS local_planner_python
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# target_compile_definitions(local_planner_python
#                            PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})


pybind_add_module(logger MODULE src/logger.cpp)

catkin_install_python(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../gym_scripts/python/examples/hello_world.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS logger
  LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#############
## Testing ##
#############

## Google test file for local_planner library
catkin_add_gtest(quadruped_mpc_test test/test_quadruped_mpc.cpp)
target_link_libraries(quadruped_mpc_test quadruped_mpc ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

catkin_add_gtest(local_footstep_planner_test test/test_local_footstep_planner.cpp)
target_link_libraries(local_footstep_planner_test local_planner ${catkin_LIBRARIES})

catkin_add_gtest(local_planner_test test/test_local_planner.cpp)
target_link_libraries(local_planner_test local_planner ${catkin_LIBRARIES})

catkin_add_gtest(mpc_controller_test test/test_mpc_controller.cpp)
target_link_libraries(mpc_controller_test mpc_controller ${catkin_LIBRARIES})


