cmake_minimum_required(VERSION 3.0.2)
project(controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES spirit-component-template
#  CATKIN_DEPENDS roscpp std_msgs
#	DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library for control
add_library(controller
	src/controller.cpp
)
add_dependencies(controller ${controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(controller_node src/controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(controller_node
  controller
	${catkin_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for controller library
catkin_add_gtest(controller_test test/test_controller.cpp)
target_link_libraries(controller_test ${catkin_LIBRARIES} controller)


