cmake_minimum_required(VERSION 3.0.2)
project(contact_detection)

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

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES contact_detection
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
)

## Declare a C++ library
add_library(contact_detection
	src/contact_detection.cpp
)
add_dependencies(contact_detection ${contact_detection_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(contact_detection_node src/contact_detection_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(contact_detection_node
	contact_detection
	${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(contact_detection_test test/test_contact_detection.cpp)
target_link_libraries(contact_detection_test ${catkin_LIBRARIES} contact_detection)

