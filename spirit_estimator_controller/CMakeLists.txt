cmake_minimum_required(VERSION 3.0.2)
project(spirit_estimator_controller)

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

## Declare a C++ library for state estimation
add_library(spirit_estimator
  src/spirit_estimator.cpp
)
add_dependencies(spirit_estimator ${spirit_estimator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ library for control
add_library(spirit_controller
	src/spirit_controller.cpp
)
add_dependencies(spirit_controller ${spirit_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(spirit_estimator_controller_node src/spirit_estimator_controller_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(spirit_estimator_controller_node
	spirit_estimator
  spirit_controller
	${catkin_LIBRARIES}
	${YAML_CPP_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for spirit_estimator library
catkin_add_gtest(spirit_estimator_test test/test_spirit_estimator.cpp)
target_link_libraries(spirit_estimator_test ${catkin_LIBRARIES} spirit_estimator)

## Google test file for spirit_controller library
catkin_add_gtest(spirit_controller_test test/test_spirit_controller.cpp)
target_link_libraries(spirit_controller_test ${catkin_LIBRARIES} spirit_controller)


