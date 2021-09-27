cmake_minimum_required(VERSION 3.0.2)
project(open_loop_controller)

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
  spirit_utils
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs spirit_msgs spirit_utils
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
add_library(open_loop_controller
	src/open_loop_controller.cpp
	src/silly_walk_template.cpp
)
add_dependencies(open_loop_controller ${open_loop_controller} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(open_loop_controller_node src/open_loop_controller_node.cpp)
add_executable(silly_walk_template_node src/silly_walk_template_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(open_loop_controller_node
  open_loop_controller
	${catkin_LIBRARIES}
)

target_link_libraries(silly_walk_template_node
  open_loop_controller
	${catkin_LIBRARIES}
)
#############
## Testing ##
#############

## Google test file for clark_trot_controller library
catkin_add_gtest(open_loop_controller_test test/test_open_loop_controller.cpp)
target_link_libraries(open_loop_controller_test ${catkin_LIBRARIES} open_loop_controller)


