cmake_minimum_required(VERSION 3.0.2)
project(leg_controller)

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
	src/leg_controller.cpp
  src/leg_controller_template.cpp
	src/inverse_dynamics.cpp
	src/grf_pid_controller.cpp
	src/joint_controller.cpp
  src/leg_overrider.cpp
)
add_dependencies(leg_controller ${leg_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
  test/test_inverse_dynamics.cpp)
target_link_libraries(leg_controller_test leg_controller ${catkin_LIBRARIES})


