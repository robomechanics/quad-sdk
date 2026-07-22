cmake_minimum_required(VERSION 3.8)
project(quad_perf_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quad_utils REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_library(cmd_vel_publisher
  src/cmd_vel_publisher.cpp
)

ament_target_dependencies(cmd_vel_publisher
  rclcpp
  geometry_msgs
  quad_utils
)

add_executable(cmd_vel_publisher_node
                src/cmd_vel_publisher_node.cpp)
target_link_libraries(cmd_vel_publisher_node cmd_vel_publisher)

ament_target_dependencies(cmd_vel_publisher_node
  rclcpp
  geometry_msgs
  quad_utils
)

install(TARGETS
  cmd_vel_publisher
  cmd_vel_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Python test-iteration tooling (moved here from the former quad_training pkg).
# Installed as executables so `ros2 run quad_perf_tests <node>` works.
install(PROGRAMS scripts/time_sync_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME time_sync_node)
install(PROGRAMS scripts/episode_monitor_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME episode_monitor_node)
install(PROGRAMS scripts/wait_for_robot_node.py
  DESTINATION lib/${PROJECT_NAME} RENAME wait_for_robot_node)
install(PROGRAMS scripts/run_iterations.py
  DESTINATION lib/${PROJECT_NAME} RENAME run_iterations)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_cmd_vel_publisher_test
    test/test_cmd_vel_publisher.cpp
  )
  if(TARGET ${PROJECT_NAME}_cmd_vel_publisher_test)
    target_compile_definitions(${PROJECT_NAME}_cmd_vel_publisher_test PRIVATE
      QUAD_PERF_TESTING
    )
    target_link_libraries(${PROJECT_NAME}_cmd_vel_publisher_test
      cmd_vel_publisher
    )
    target_include_directories(${PROJECT_NAME}_cmd_vel_publisher_test PRIVATE
      include
    )
    ament_target_dependencies(${PROJECT_NAME}_cmd_vel_publisher_test
      rclcpp
      geometry_msgs
      quad_utils
    )
    set_tests_properties(${PROJECT_NAME}_cmd_vel_publisher_test PROPERTIES
      ENVIRONMENT
        "QUAD_PERF_TEST_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/cmd_vel_publisher.yaml;QUAD_PERF_TEST_TOPIC_PARAMS=${CMAKE_CURRENT_SOURCE_DIR}/config/cmd_vel_publisher_topics.yaml"
    )
  endif()

  ament_add_pytest_test(${PROJECT_NAME}_python_tests
    test/test_quad_perf_scripts.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_SOURCE_DIR}/launch
    TIMEOUT 30
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()

ament_package()
