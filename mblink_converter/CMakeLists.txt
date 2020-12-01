cmake_minimum_required(VERSION 3.0.2)
project(mblink_converter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  spirit_msgs
)

## Setup MBLINK and MAVLINK
set(CMAKE_CXX_STANDARD 17)
set(MBLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/mblink)

message(STATUS "Using architecture ${CMAKE_SYSTEM_PROCESSOR}")

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(GRMBLINK libgrmblink_xavier.a)
else()
  set(GRMBLINK libgrmblink_linux.a)
endif()

include_directories(${MBLINK_DIR}/include)
link_directories(${MBLINK_DIR}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../external/SpiritSDK/SDK/thirdparty)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES mblink_converter
  CATKIN_DEPENDS roscpp spirit_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_library(mblink_converter
  src/mblink_converter.cpp
)
add_dependencies(mblink_converter ${mblink_converter_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mblink_converter
  ${GRMBLINK}
  ${catkin_LIBRARIES}
)

add_executable(mblink_converter_node src/mblink_converter_node.cpp)
add_dependencies(mblink_converter_node ${mblink_converter_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mblink_converter_node
  ${GRMBLINK}
  mblink_converter
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Google test file for mblibk_converter library
catkin_add_gtest(mblink_converter_test test/test_mblink_converter.cpp)
target_link_libraries(mblink_converter_test ${catkin_LIBRARIES} mblink_converter)
