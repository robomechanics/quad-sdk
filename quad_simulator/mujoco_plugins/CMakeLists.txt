cmake_minimum_required(VERSION 3.8)
project(mujoco_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

add_executable(mujoco_ground_truth_node
  src/mujoco_ground_truth_node.cpp
)

ament_target_dependencies(mujoco_ground_truth_node
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  quad_msgs
  quad_utils
)

target_link_libraries(mujoco_ground_truth_node
  pinocchio::pinocchio_parsers
)

install(TARGETS
  mujoco_ground_truth_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
