cmake_minimum_required(VERSION 3.8)
project(gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quad_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

# --- Gazebo Sim system plugin: ground-truth estimator ---
add_library(gazebo_estimator_plugin SHARED
  src/gazebo_estimator_plugin.cpp
)

target_include_directories(gazebo_estimator_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gazebo_estimator_plugin
  rclcpp
  std_msgs
  quad_utils
  quad_msgs
  gz-sim8
  gz-plugin2
)

install(TARGETS
  gazebo_estimator_plugin
  DESTINATION lib
)

# --- Contact state publisher node (consumes ros_gz_interfaces contact msgs) ---
add_executable(contact_state_publisher_node
  src/contact_state_publisher_node.cpp
  src/contact_state_publisher.cpp
)

target_include_directories(contact_state_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(contact_state_publisher_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  quad_utils
  quad_msgs
  ros_gz_interfaces
)

install(TARGETS contact_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()

ament_package()
