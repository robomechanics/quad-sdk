cmake_minimum_required(VERSION 3.8)
project(quad_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_executable(mode_supervisor src/mode_supervisor.cpp)
ament_target_dependencies(mode_supervisor
  rclcpp
  std_msgs
  quad_msgs
  controller_manager_msgs)

install(TARGETS mode_supervisor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
