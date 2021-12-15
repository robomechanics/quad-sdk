cmake_minimum_required(VERSION 3.0.2)
project(leg_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
  
## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Setup MBLINK and MAVLINK
set(CMAKE_CXX_STANDARD 17)
set(MBLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/mblink/)

message(STATUS "Using architecture ${CMAKE_SYSTEM_PROCESSOR}")

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(GRMBLINK libgrmblink_tx2.a)
else()
  set(GRMBLINK libgrmblink_linux.a)
endif()

include_directories(${MBLINK_DIR}/include)
link_directories(${MBLINK_DIR}/lib)

find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quad_msgs
  quad_utils
  eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs quad_msgs quad_utils eigen_conversions
  DEPENDS EIGEN3
)

###########
##  Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library for control
add_library(leg_controller
	src/leg_controller_interface.cpp
  src/leg_controller_template.cpp
	src/inverse_dynamics.cpp
	src/grf_pid_controller.cpp
	src/joint_controller.cpp
  src/leg_overrider.cpp
  src/mblink_converter.cpp
)
add_dependencies(leg_controller ${leg_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(leg_controller
  ${GRMBLINK}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(leg_controller_node src/leg_controller_node.cpp)
add_executable(leg_overrider_node src/leg_overrider_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(leg_controller_node
  leg_controller
	${catkin_LIBRARIES}
)
target_link_libraries(leg_overrider_node
  leg_controller
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for leg_controller library
catkin_add_gtest(leg_controller_test
  test/test_leg_controller.cpp
  test/test_mblink_converter.cpp)
target_link_libraries(leg_controller_test leg_controller ${catkin_LIBRARIES})


