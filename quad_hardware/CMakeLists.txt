cmake_minimum_required(VERSION 3.8)
project(quad_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

# Unitree SDK2 — same external tree and manual linking robot_driver uses.
set(UNITREE_SDK2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/unitree_sdk2)
# MBLink (Ghost Robotics Spirit) — same external tree robot_driver uses.
set(MBLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/mblink)

message(STATUS "quad_hardware: architecture ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  set(UNITREE_LIB_DIR ${UNITREE_SDK2_DIR}/lib/aarch64)
  set(UNITREE_THIRD_PARTY_DIR ${UNITREE_SDK2_DIR}/thirdparty/lib/aarch64)
  set(GRMBLINK libgrmblink_tx2.a)
else()
  set(UNITREE_LIB_DIR ${UNITREE_SDK2_DIR}/lib/x86_64)
  set(UNITREE_THIRD_PARTY_DIR ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64)
  set(GRMBLINK libgrmblink_linux.a)
endif()

add_library(quad_hardware SHARED
  src/unitree_system.cpp
  src/spirit_system.cpp)

target_include_directories(quad_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UNITREE_SDK2_DIR}/include
  ${UNITREE_SDK2_DIR}/thirdparty/include
  ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
  ${MBLINK_DIR}/include
  ${EIGEN3_INCLUDE_DIR})

target_link_directories(quad_hardware PUBLIC
  ${UNITREE_LIB_DIR}
  ${UNITREE_THIRD_PARTY_DIR}
  ${MBLINK_DIR}/lib)

target_link_libraries(quad_hardware
  unitree_sdk2 ddsc ddscxx ${GRMBLINK} pthread)

ament_target_dependencies(quad_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3)

# Export the SystemInterface plugin to pluginlib.
pluginlib_export_plugin_description_file(hardware_interface quad_hardware_plugins.xml)

install(TARGETS quad_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(FILES quad_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(quad_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
