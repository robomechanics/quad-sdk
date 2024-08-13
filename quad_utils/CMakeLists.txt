cmake_minimum_required(VERSION 3.5)
project(quad_utils)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)

# Search for the RBDL include directory and library
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

# Package configuration
ament_export_include_directories(include ${PYTHON_INCLUDE_DIRS})
ament_export_libraries(quad_utils)
ament_export_dependencies(rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs quad_msgs grid_map_core grid_map_ros grid_map_pcl Eigen3)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
# LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
)

# Add Library
add_library(quad_utils
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/terrain_map_publisher.cpp
  src/trajectory_publisher.cpp
  src/mesh_to_grid_map_converter.cpp
  src/math_utils.cpp
  src/ros_utils.cpp
  src/quad_kd.cpp
  src/remote_heartbeat.cpp
)

ament_target_dependencies(quad_utils rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs quad_msgs grid_map_core grid_map_ros grid_map_pcl Eigen3)

target_link_libraries(quad_utils
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
)

# Declare C++ executables
add_executable(rviz_interface_node src/rviz_interface_node.cpp)
add_executable(terrain_map_publisher_node src/terrain_map_publisher_node.cpp)
add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
add_executable(mesh_to_grid_map_node src/mesh_to_grid_map_node.cpp)
add_executable(remote_heartbeat_node src/remote_heartbeat_node.cpp)

# Link libraries
target_link_libraries(rviz_interface_node
  quad_utils
)

target_link_libraries(terrain_map_publisher_node
  quad_utils
)

target_link_libraries(trajectory_publisher_node
  quad_utils
)

target_link_libraries(mesh_to_grid_map_node
  quad_utils
)

target_link_libraries(remote_heartbeat_node
  quad_utils
)

# Install
install(TARGETS
  quad_utils
  rviz_interface_node
  terrain_map_publisher_node
  trajectory_publisher_node
  mesh_to_grid_map_node
  remote_heartbeat_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing
ament_add_gtest(quad_utils-test 
  test/test_rviz_interface.cpp
  test/test_terrain_map_publisher.cpp
  test/test_fast_terrain_map.cpp
  test/test_quad_kd.cpp
  test/test_eigen.cpp
  test/test_math_utils.cpp
  test/test_main.cpp)

target_link_libraries(quad_utils-test quad_utils ${PYTHON_LIBRARIES})

ament_package()
