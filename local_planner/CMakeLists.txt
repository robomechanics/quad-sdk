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
  spirit_msgs
  spirit_utils
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PythonLibs 2.7)

set(QPOASES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../external/qpOASES")
set(QPOASES_INCLUDE_DIRS "${QPOASES_DIR}/include")
set(QPOASES_LIBRARIES "${QPOASES_DIR}/build/libs")
find_library(QPOASES NAMES qpOASES PATHS ${QPOASES_LIBRARIES} REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs spirit_msgs spirit_utils
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
)

add_library(quadruped_mpc
   src/quadruped_mpc.cpp
)
target_link_libraries(quadruped_mpc OsqpEigen::OsqpEigen )

## Declare a C++ library for control
add_library(local_planner
  src/local_planner.cpp
  src/local_body_planner.cpp
  src/local_footstep_planner.cpp
)
target_link_libraries(local_planner quadruped_mpc)
add_dependencies(local_planner ${local_planner_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(local_planner_node src/local_planner_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(local_planner_node
  local_planner
  ${catkin_LIBRARIES}
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


