cmake_minimum_required(VERSION 3.0.2)
project(clark_trot_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  spirit_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs spirit_msgs
  DEPENDS 
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
add_library(clark_trot_controller
	src/clark_trot_controller.cpp
)
add_dependencies(clark_trot_controller ${clark_trot_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(clark_trot_controller_node src/clark_trot_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(clark_trot_controller_node
  clark_trot_controller
	${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for clark_trot_controller library
catkin_add_gtest(clark_trot_controller_test test/test_clark_trot_controller.cpp)
target_link_libraries(clark_trot_controller_test ${catkin_LIBRARIES} clark_trot_controller)


