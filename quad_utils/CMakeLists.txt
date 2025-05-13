cmake_minimum_required(VERSION 3.8)
project(quad_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs)
find_package(nav2_msgs)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(mesh_to_grid_map_node 
  src/mesh_to_grid_map_node.cpp
  src/mesh_to_grid_map_converter.cpp)

ament_target_dependencies(mesh_to_grid_map_node 
  rclcpp 
  grid_map_msgs 
  grid_map_core 
  grid_map_pcl 
  grid_map_ros
  quad_msgs 
  pcl_msgs
  pcl_conversions
  nav_msgs
  nav2_msgs
  cv_bridge
  visualization_msgs
)

target_include_directories(mesh_to_grid_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(mesh_to_grid_map_node
  ${PCL_LIBRARIES}
)

# Install Executables
install(TARGETS
  mesh_to_grid_map_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install YAML Files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
