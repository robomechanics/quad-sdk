cmake_minimum_required(VERSION 3.8)
project(quad_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs)
find_package(nav2_msgs)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Python3 REQUIRED)

find_package(pluginlib REQUIRED)
find_package(filters REQUIRED)
find_package(tf2_ros REQUIRED)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Search for the RBDL include directory and library
find_package(RBDL REQUIRED COMPONENTS URDFREADER)

add_library(quad_utils
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/mesh_to_grid_map_converter.cpp
  src/math_utils.cpp
  src/ros_utils.cpp
  src/quad_kd.cpp
  # src/trajectory_publisher.cpp
  # Add other reusable sources here if needed
)

target_include_directories(quad_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${RBDL_URDFREADER_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)

# Make sure Quad-Utils can access RBDL Libraries
target_link_libraries(quad_utils
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
)

add_executable(mesh_to_grid_map_node 
  src/mesh_to_grid_map_node.cpp
  src/mesh_to_grid_map_converter.cpp)

add_executable(rviz_interface_node
  src/rviz_interface_node.cpp
  src/rviz_interface.cpp)

# add_executable(trajectory_publisher_node
#   src/trajectory_publisher_node.cpp
#   src/trajectory_publisher.cpp
#   )
add_executable(grid_map_filters_demo
  src/filters_demo.cpp
  src/filters_demo_node.cpp
)

target_include_directories(rviz_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# target_include_directories(trajectory_publisher_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

target_include_directories(grid_map_filters_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

  
target_link_libraries(mesh_to_grid_map_node
  quad_utils
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${PYTHON_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

target_link_libraries(rviz_interface_node
  quad_utils
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}

)

ament_target_dependencies(quad_utils 
  rclcpp 
  std_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  grid_map_core
  grid_map_ros 
  grid_map_pcl
  grid_map_msgs  
  quad_msgs 
  pcl_msgs
  pcl_conversions
  Eigen3
  tf2
  tf2_geometry_msgs
  cv_bridge
)

ament_target_dependencies(mesh_to_grid_map_node rclcpp)

ament_target_dependencies(rviz_interface_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  quad_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ament_target_dependencies(trajectory_publisher_node
#   rclcpp
#   quad_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   visualization_msgs
# )
ament_target_dependencies(grid_map_filters_demo
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pluginlib
  filters
)

install(TARGETS
  grid_map_filters_demo
  DESTINATION lib/${PROJECT_NAME}
)


# Install Libraries and Executables
install(TARGETS quad_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS
  mesh_to_grid_map_node
  DESTINATION lib/${PROJECT_NAME}

)

install(TARGETS
  mesh_to_grid_map_node
  rviz_interface_node
  # trajectory_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install YAML Files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Includes
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(FILES cmake/FindRBDL.cmake 
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_libraries(quad_utils ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY})
ament_export_dependencies(
  rclcpp std_msgs nav_msgs nav2_msgs sensor_msgs geometry_msgs visualization_msgs
  grid_map_core grid_map_ros grid_map_pcl grid_map_msgs quad_msgs
  pcl_msgs pcl_conversions Eigen3 tf2 tf2_geometry_msgs cv_bridge pluginlib filters
)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
#   ament_add_gtest(test_quad_kd
#     test/test_quad_kd.cpp
#   )
#   set_tests_properties(test_quad_kd PROPERTIES DISABLED TRUE)
#   ament_target_dependencies(test_quad_kd
#     rclcpp
#     std_msgs
#     grid_map_core
#     Eigen3
#     tf2
#     quad_msgs
#   )
#   target_link_libraries(test_quad_kd quad_utils)
#   target_include_directories(test_quad_kd PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   )
#   add_launch_test(
#     launch/test_quad_kd.py
#     TARGET test_quad_kd_launch
#     TIMEOUT 150
#   )
  
#   install(TARGETS
#     test_quad_kd
#     DESTINATION lib/${PROJECT_NAME}
#   )
#   install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )
endif()

ament_package()
