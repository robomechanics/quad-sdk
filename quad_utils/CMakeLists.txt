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
find_package(pinocchio REQUIRED)

find_package(pluginlib REQUIRED)
find_package(filters REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(mujoco_vendor QUIET)
find_package(glfw3 QUIET)

add_library(quad_utils
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/mesh_to_grid_map_converter.cpp
  src/math_utils.cpp
  src/ros_utils.cpp
  src/quad_kd2.cpp
  # src/trajectory_publisher.cpp
)

# Pinocchio + Eigen template expansion needs more RAM at higher optimization.
# -O2 requires ~10-12GB; the Jetson Orin (16GB) can handle it.
if(CMAKE_BUILD_TYPE MATCHES "Release|RelWithDebInfo")
  set_source_files_properties(src/quad_kd2.cpp
    PROPERTIES COMPILE_OPTIONS "-O2"
  )
endif()

target_include_directories(quad_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(mesh_to_grid_map_node 
  src/mesh_to_grid_map_node.cpp
  src/mesh_to_grid_map_converter.cpp)

add_executable(mjcf_to_grid_map_node
  src/mjcf_to_grid_map_node.cpp
  src/mjcf_to_grid_map_converter.cpp)

add_executable(rviz_interface_node
  src/rviz_interface_node.cpp
  src/rviz_interface.cpp)

# add_executable(trajectory_publisher_node
#   src/trajectory_publisher_node.cpp
#   src/trajectory_publisher.cpp
#   )

add_executable(remote_heartbeat_node
  src/remote_heartbeat_node.cpp
  src/remote_heartbeat.cpp)

target_include_directories(remote_heartbeat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(grid_map_filters_demo
  src/filters_demo.cpp
  src/filters_demo_node.cpp
)

target_include_directories(rviz_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_include_directories(mjcf_to_grid_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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


target_link_libraries(mjcf_to_grid_map_node
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

# target_link_libraries(trajectory_publisher_node
#   quad_utils
#   ${tf2_LIBRARIES}
#   ${tf2_geometry_msgs_LIBRARIES}
# )


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
  pinocchio
)

ament_target_dependencies(mesh_to_grid_map_node rclcpp)


ament_target_dependencies(mjcf_to_grid_map_node
  rclcpp
  std_msgs
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
#   grid_map_core  
# )

target_link_libraries(remote_heartbeat_node
  quad_utils
)

ament_target_dependencies(remote_heartbeat_node
  rclcpp
  std_msgs
)

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


if(mujoco_vendor_FOUND AND glfw3_FOUND)
  add_executable(mujoco_recorder_node src/mujoco_recorder_node.cpp)
  set_target_properties(mujoco_recorder_node PROPERTIES OUTPUT_NAME mujoco_recorder)
  target_link_libraries(mujoco_recorder_node
    mujoco::mujoco
    glfw
  )
  ament_target_dependencies(mujoco_recorder_node
    rclcpp
    sensor_msgs
    nav_msgs
  )
  install(TARGETS mujoco_recorder_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "mujoco_recorder_node: skipping (mujoco_vendor or glfw3 not found)")
endif()

# install(TARGETS
#   trajectory_publisher_node
#   DESTINATION lib/${PROJECT_NAME}
# )

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
  mjcf_to_grid_map_node
  rviz_interface_node
  remote_heartbeat_node
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

ament_export_include_directories(include)
ament_export_libraries(quad_utils)
ament_export_dependencies(
  rclcpp std_msgs nav_msgs nav2_msgs sensor_msgs geometry_msgs visualization_msgs
  grid_map_core grid_map_ros grid_map_pcl grid_map_msgs quad_msgs
  pcl_msgs pcl_conversions Eigen3 tf2 tf2_geometry_msgs cv_bridge pluginlib filters
  pinocchio
)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(grid_map_core REQUIRED)
  # find_package(launch_testing_ament_cmake REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_add_gtest(quad_utils_test
    test/test_quad_kd.cpp
    test/test_rviz_interface.cpp
    test/test_main.cpp
    test/test_math_utils.cpp
  )

  if(TARGET quad_utils_test)
    target_link_libraries(quad_utils_test
      quad_utils
    )

    ament_target_dependencies(quad_utils_test
      rclcpp
      std_msgs
      grid_map_core
      tf2
      quad_msgs
    )

    target_include_directories(quad_utils_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  endif()
endif()

ament_package()
