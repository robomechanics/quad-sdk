cmake_minimum_required(VERSION 3.0.2)
project(ekf_estimator)

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
  sensor_msgs
  spirit_msgs
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs spirit_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library for state estimation
add_library(ekf_estimator
  src/ekf_estimator.cpp
)
add_dependencies(ekf_estimator ${ekf_estimator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ekf_estimator_node src/ekf_estimator_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ekf_estimator_node
	ekf_estimator
	${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for ekf_estimator library
catkin_add_gtest(ekf_estimator_test test/test_ekf_estimator.cpp)
target_link_libraries(ekf_estimator_test ${catkin_LIBRARIES} ekf_estimator)