cmake_minimum_required(VERSION 3.0.2)
project(quad_utils)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

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
  quad_msgs
  grid_map_core
  grid_map_ros
  grid_map_pcl
)

find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

# Search for the RBDL include directory and library
FIND_PACKAGE (RBDL COMPONENTS URDFREADER REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${PYTHON_INCLUDE_DIRS}
  LIBRARIES quad_utils
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs quad_msgs grid_map_core grid_map_ros grid_map_pcl
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
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

add_library(quad_utils
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/terrain_map_publisher.cpp
  src/trajectory_publisher.cpp
  src/mesh_to_grid_map_converter.cpp
  src/math_utils.cpp
  src/ros_utils.cpp
  src/quad_kd.cpp
  src/trajectory_publisher.cpp
  src/remote_heartbeat.cpp
)
add_dependencies(quad_utils ${quad_utils_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_utils)
target_include_directories(quad_utils INTERFACE ${quad_utils_INCLUDE_DIRS})

# And link the library against the executable
TARGET_LINK_LIBRARIES ( quad_utils
	${RBDL_LIBRARY}
	${RBDL_URDFREADER_LIBRARY}
	)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rviz_interface_node src/rviz_interface_node.cpp)
add_executable(terrain_map_publisher_node src/terrain_map_publisher_node.cpp)
add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
add_executable(mesh_to_grid_map_node src/mesh_to_grid_map_node.cpp)
add_executable(remote_heartbeat_node src/remote_heartbeat_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rviz_interface_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(terrain_map_publisher_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_publisher_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(mesh_to_grid_map_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(remote_heartbeat_node
  quad_utils
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(quad_utils-test 
  test/test_rviz_interface.cpp
  test/test_terrain_map_publisher.cpp
  test/test_fast_terrain_map.cpp
  test/test_quad_kd.cpp
  test/test_eigen.cpp
  test/test_math_utils.cpp
  test/test_main.cpp)
target_link_libraries(quad_utils-test quad_utils ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
