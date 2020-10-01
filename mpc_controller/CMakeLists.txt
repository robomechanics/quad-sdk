cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

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
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs
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

## Declare a C++ library for control
add_library(mpc_controller
	src/mpc_controller.cpp
)
add_dependencies(mpc_controller ${mpc_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(mpc_controller_node src/mpc_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mpc_controller_node
  mpc_controller
	${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for mpc_controller library
catkin_add_gtest(mpc_controller_test test/test_mpc_controller.cpp)
target_link_libraries(mpc_controller_test ${catkin_LIBRARIES} mpc_controller)


