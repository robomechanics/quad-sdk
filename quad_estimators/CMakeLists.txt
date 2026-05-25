cmake_minimum_required(VERSION 3.8)
project(quad_estimators)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(quad_utils REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Estimator library (ported math, fully decoupled from robot_driver)
set(QUAD_ESTIMATORS_SOURCES
  src/state_estimator.cpp
  src/comp_filter_estimator.cpp
  src/ekf_estimator.cpp
)

add_library(quad_estimators SHARED ${QUAD_ESTIMATORS_SOURCES})

target_include_directories(quad_estimators PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(quad_estimators
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  quad_msgs
  quad_utils
  Eigen3
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

# Complementary filter estimator node
add_executable(comp_filter_estimator_node src/comp_filter_estimator_node.cpp)
target_link_libraries(comp_filter_estimator_node quad_estimators)
ament_target_dependencies(comp_filter_estimator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  quad_msgs
  quad_utils
)

# EKF estimator node
add_executable(ekf_estimator_node src/ekf_estimator_node.cpp)
target_link_libraries(ekf_estimator_node quad_estimators)
ament_target_dependencies(ekf_estimator_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  quad_msgs
  quad_utils
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  quad_estimators
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  comp_filter_estimator_node
  ekf_estimator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
