cmake_minimum_required(VERSION 3.0.2)
project(realsense_grid_map)

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Setup MBLINK and MAVLINK
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
find_package(catkin REQUIRED COMPONENTS
  grid_map_cv
  grid_map_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS grid_map_cv grid_map_ros roscpp rospy sensor_msgs std_msgs cv_bridge
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/realsense_grid_map_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME realsense_grid_map_node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)