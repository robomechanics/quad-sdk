cmake_minimum_required(VERSION 3.8)
project(body_force_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Both robot dynamics variants + the runtime dispatcher.
# Selection between them is done at RUNTIME via the `robot_type` ROS parameter
# ("spirit" default / "go2"); see body_force_estimator_dynamics.hpp.
add_library(${PROJECT_NAME}
  src/body_force_estimator.cpp
  src/body_force_estimator_dynamics.cpp           # dispatcher / pointers
  src/spirit_body_force_estimator_dynamics.cpp    # spirit_impl namespace
  src/go2_body_force_estimator_dynamics.cpp       # go2_impl namespace
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_node src/body_force_estimator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/leg_flail.py
  scripts/path_following.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs quad_msgs quad_utils)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
