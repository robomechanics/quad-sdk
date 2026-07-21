cmake_minimum_required(VERSION 3.8)
project(local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nmpc_controller REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  /usr/local/include/coin-or
)

link_directories(/usr/local/lib)

add_library(local_planner
  src/local_planner.cpp
  src/local_footstep_planner.cpp
)

add_executable(local_planner_node src/local_planner_node.cpp)
target_link_libraries(local_planner_node local_planner)

ament_target_dependencies(local_planner
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
  grid_map_core
  grid_map_ros
  tf2_eigen
  nmpc_controller
  tf2_geometry_msgs
  tf2
)

ament_target_dependencies(local_planner_node
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
  grid_map_core
  grid_map_ros
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

install(TARGETS
  local_planner
  local_planner_node
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


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_footstep_planner.cpp
    test/test_local_planner.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE
      LOCAL_PLANNER_TESTING
    )
    target_link_libraries(${PROJECT_NAME}_test local_planner)
    target_include_directories(${PROJECT_NAME}_test PRIVATE
      include
      ${EIGEN3_INCLUDE_DIRS}
    )
    ament_target_dependencies(${PROJECT_NAME}_test
      rclcpp
      std_msgs
      quad_msgs
      quad_utils
      grid_map_core
      grid_map_ros
      geometry_msgs
      tf2_eigen
      nmpc_controller
      tf2_geometry_msgs
      tf2
    )
    set_tests_properties(${PROJECT_NAME}_test PROPERTIES
      ENVIRONMENT
        "LOCAL_PLANNER_TEST_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/local_planner.yaml;LOCAL_PLANNER_TOPIC_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/local_planner_topics.yaml;LOCAL_PLANNER_ROBOT_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/../quad_utils/config/spirit.yaml;LOCAL_PLANNER_NMPC_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/../nmpc_controller/config/nmpc_controller.yaml;LOCAL_PLANNER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/.."
      )
    endif()

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
