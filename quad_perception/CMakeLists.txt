cmake_minimum_required(VERSION 3.8)
project(quad_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Unitree SDK2 setup (mirrors robot_driver/CMakeLists.txt)
set(UNITREE_SDK2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/unitree_sdk2)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  set(UNITREE_LIB_DIR ${UNITREE_SDK2_DIR}/lib/aarch64)
  set(UNITREE_THIRD_PARTY_DIR ${UNITREE_SDK2_DIR}/thirdparty/lib/aarch64)
else()
  set(UNITREE_LIB_DIR ${UNITREE_SDK2_DIR}/lib/x86_64)
  set(UNITREE_THIRD_PARTY_DIR ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64)
endif()

# Conversion library (pure DDS->ROS translation; no DDS runtime, no rclcpp).
# Split out so tests can link only this without pulling in the DDS runtime.
add_library(unitree_pointcloud_conversion SHARED
  src/unitree_pointcloud_conversion.cpp
)
target_include_directories(unitree_pointcloud_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UNITREE_SDK2_DIR}/include
  ${UNITREE_SDK2_DIR}/thirdparty/include
  ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
)
ament_target_dependencies(unitree_pointcloud_conversion
  sensor_msgs
  std_msgs
  builtin_interfaces
)

# Bridge library — owns the ChannelSubscriber and the rclcpp::Node.
add_library(unitree_pointcloud_bridge SHARED
  src/unitree_pointcloud_bridge.cpp
)
target_include_directories(unitree_pointcloud_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UNITREE_SDK2_DIR}/include
  ${UNITREE_SDK2_DIR}/thirdparty/include
  ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
)
target_link_directories(unitree_pointcloud_bridge PUBLIC
  ${UNITREE_LIB_DIR}
  ${UNITREE_THIRD_PARTY_DIR}
)
target_link_libraries(unitree_pointcloud_bridge
  unitree_pointcloud_conversion
  unitree_sdk2 ddsc ddscxx pthread
)
ament_target_dependencies(unitree_pointcloud_bridge
  rclcpp
  sensor_msgs
  std_msgs
  builtin_interfaces
)

add_executable(unitree_pointcloud_bridge_node src/unitree_pointcloud_bridge_node.cpp)
target_link_libraries(unitree_pointcloud_bridge_node unitree_pointcloud_bridge)
ament_target_dependencies(unitree_pointcloud_bridge_node rclcpp)

# Point cloud -> grid_map elevation layer. Source agnostic: takes any
# PointCloud2, so it serves the Livox and the Unitree bridge alike.
add_library(pointcloud_to_gridmap SHARED
  src/pointcloud_to_gridmap.cpp
)
target_include_directories(pointcloud_to_gridmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_to_gridmap
  rclcpp
  sensor_msgs
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  tf2_ros
  tf2_sensor_msgs
)

add_executable(pointcloud_to_gridmap_node src/pointcloud_to_gridmap_node.cpp)
target_link_libraries(pointcloud_to_gridmap_node pointcloud_to_gridmap)
ament_target_dependencies(pointcloud_to_gridmap_node rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  unitree_pointcloud_conversion
  unitree_pointcloud_bridge
  unitree_pointcloud_bridge_node
  pointcloud_to_gridmap
  pointcloud_to_gridmap_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Skip lint hooks that require repo-wide setup we don't have here.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  ament_add_gtest(test_unitree_pointcloud_conversion
    test/test_unitree_pointcloud_conversion.cpp
  )
  target_link_libraries(test_unitree_pointcloud_conversion
    unitree_pointcloud_conversion
  )
  ament_target_dependencies(test_unitree_pointcloud_conversion
    sensor_msgs
    std_msgs
    builtin_interfaces
  )
endif()

ament_package()
