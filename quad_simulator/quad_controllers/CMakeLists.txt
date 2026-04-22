cmake_minimum_required(VERSION 3.8)
project(quad_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)

add_library(quad_controller SHARED
  src/controller_plugin.cpp
)

target_include_directories(quad_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(quad_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  angles
  realtime_tools
  urdf
  quad_msgs
  quad_utils
)

pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)

install(TARGETS
  quad_controller
  DESTINATION lib
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
