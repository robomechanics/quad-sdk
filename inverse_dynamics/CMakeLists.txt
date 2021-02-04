cmake_minimum_required(VERSION 3.0.2)
project(inverse_dynamics)

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

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs spirit_msgs spirit_utils
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
add_library(inverse_dynamics
	src/inverse_dynamics.cpp
)
add_dependencies(inverse_dynamics ${inverse_dynamics_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(inverse_dynamics_node src/inverse_dynamics_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(inverse_dynamics_node
  inverse_dynamics
	${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for inverse_dynamics library
catkin_add_gtest(inverse_dynamics_test test/test_inverse_dynamics.cpp)
target_link_libraries(inverse_dynamics_test ${catkin_LIBRARIES} inverse_dynamics)


