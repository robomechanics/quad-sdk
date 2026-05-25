cmake_minimum_required(VERSION 3.8)
project(quad_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ONNX Runtime is optional — only needed for the learned policy controller
find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS /usr/local/lib /usr/local/lib64)

# A shared controller base + four switchable ros2_control controllers
# (leg-control / pose / safety / transition) + the leg-control laws the
# LegControlController delegates to (faithful ports of robot_driver's
# LegController hierarchy). The learned_policy law is internally guarded by
# HAS_ONNXRUNTIME, so it is always listed and degrades to an empty translation
# unit when ONNX is absent.
set(QUAD_CONTROLLERS_SOURCES
  src/quad_controller_base.cpp
  src/leg_control_controller.cpp
  src/pose_controller.cpp
  src/safety_controller.cpp
  src/transition_controller.cpp
  src/laws/leg_controller.cpp
  src/laws/inverse_dynamics_controller.cpp
  src/laws/grf_pid_controller.cpp
  src/laws/joint_controller.cpp
  src/laws/underbrush_inverse_dynamics.cpp
  src/laws/inertia_estimation_controller.cpp
  src/laws/learned_policy.cpp)

if(ONNXRUNTIME_LIB)
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIB} — learned policy law enabled")
else()
  message(STATUS "ONNX Runtime not found — learned policy law disabled")
endif()

add_library(quad_controllers SHARED ${QUAD_CONTROLLERS_SOURCES})

target_include_directories(quad_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(quad_controllers
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  quad_msgs
  quad_utils
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  ament_index_cpp
  Eigen3)

if(ONNXRUNTIME_LIB)
  target_compile_definitions(quad_controllers PUBLIC HAS_ONNXRUNTIME)
  target_include_directories(quad_controllers PUBLIC /usr/local/include/onnxruntime)
  target_link_libraries(quad_controllers ${ONNXRUNTIME_LIB})
endif()

pluginlib_export_plugin_description_file(controller_interface quad_controllers_plugins.xml)

install(TARGETS quad_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(FILES quad_controllers_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(quad_controllers)
ament_export_dependencies(controller_interface pluginlib rclcpp rclcpp_lifecycle
  quad_msgs quad_utils std_msgs geometry_msgs sensor_msgs tf2 tf2_eigen
  tf2_geometry_msgs ament_index_cpp)

ament_package()
