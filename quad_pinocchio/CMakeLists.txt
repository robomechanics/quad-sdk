cmake_minimum_required(VERSION 3.8)
project(quad_pinocchio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(grid_map_core REQUIRED)
# find_package(grid_map_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(pinocchio REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# -------- Library: quad_kd2 --------
add_library(quad_kd2
  src/quad_kd2.cpp
)

target_include_directories(quad_kd2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Use ament for ROS deps (this uses PLAIN target_link_libraries internally)
ament_target_dependencies(quad_kd2
  rclcpp
)

# Keep PLAIN signature to avoid mixing with ament_target_dependencies
target_link_libraries(quad_kd2
  pinocchio::pinocchio
  Eigen3::Eigen
)

# -------- Executable: test_quad_kd_node --------
add_executable(test_quad_kd_node
  src/test_quad_kd_node.cpp
)

ament_target_dependencies(test_quad_kd_node
  rclcpp
)

# PLAIN signature (no PUBLIC/PRIVATE) to be consistent
target_link_libraries(test_quad_kd_node
  quad_kd2
)

# -------- Install / Export --------
install(DIRECTORY include/ DESTINATION include)

install(TARGETS quad_kd2 test_quad_kd_node
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp Eigen3 pinocchio)


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
