cmake_minimum_required(VERSION 3.8)
project(conflict_based_search)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_utils REQUIRED)
find_package(global_body_planner REQUIRED)
find_package(Eigen3 REQUIRED)

set(CBS_LIB_SOURCES
  src/conflict_based_search.cpp
)

add_library(${PROJECT_NAME} ${CBS_LIB_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  quad_msgs
  quad_utils
  global_body_planner
)

add_executable(${PROJECT_NAME}_node src/conflict_based_search_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  quad_msgs
  quad_utils
  global_body_planner
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Launch files for the CBS pipeline live alongside the rest of the
# repo's launch files in quad_utils/launch (use
# `ros2 launch quad_utils multi_robot.py`). No launch/ directory is
# installed from this package.

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  quad_msgs
  quad_utils
  global_body_planner
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_add_gtest(${PROJECT_NAME}_test
    test/test_cbs_structures.cpp
    test/test_conflicts.cpp
    test/test_geometry.cpp
    test/test_services_and_publish.cpp
    ENV
      CONFLICT_BASED_SEARCH_TEST_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/conflict_based_search.yaml
  )

  if(TARGET ${PROJECT_NAME}_test)
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE
      CONFLICT_BASED_SEARCH_TESTING
    )

    target_link_libraries(${PROJECT_NAME}_test
      ${PROJECT_NAME}
    )

    ament_target_dependencies(${PROJECT_NAME}_test
      rclcpp
      std_msgs
      geometry_msgs
      visualization_msgs
      quad_msgs
      quad_utils
      global_body_planner
    )

    target_include_directories(${PROJECT_NAME}_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
      ${EIGEN3_INCLUDE_DIRS}
    )
  endif()
endif()

ament_package()
