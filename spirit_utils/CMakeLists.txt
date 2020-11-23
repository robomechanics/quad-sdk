cmake_minimum_required(VERSION 3.0.2)
project(spirit_utils)

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
  nav_msgs
  sensor_msgs
  geometry_msgs
  spirit_msgs
  grid_map_core
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spirit_utils
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


add_library(spirit_utils
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/terrain_map_publisher.cpp
  src/ground_truth_publisher.cpp
  src/robot_interface.cpp
  src/ros_utils.cpp
)
add_dependencies(spirit_utils ${spirit_utils_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(spirit_utils)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rviz_interface_node src/rviz_interface_node.cpp)
add_executable(terrain_map_publisher_node src/terrain_map_publisher_node.cpp)
add_executable(ground_truth_publisher_node src/ground_truth_publisher_node.cpp)
add_executable(robot_interface_node src/robot_interface_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rviz_interface_node
  spirit_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(terrain_map_publisher_node
  spirit_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(ground_truth_publisher_node
  spirit_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(robot_interface_node
  spirit_utils
  ${catkin_LIBRARIES}
)



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(spirit_utils-test 
  test/test_rviz_interface.cpp
  test/test_terrain_map_publisher.cpp
  test/test_ground_truth_publisher.cpp
  test/test_robot_interface.cpp
  test/test_fast_terrain_map.cpp
  test/test_main.cpp)
target_link_libraries(spirit_utils-test spirit_utils ${catkin_LIBRARIES})

