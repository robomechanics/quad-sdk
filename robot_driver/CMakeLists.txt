cmake_minimum_required(VERSION 3.8)
project(robot_driver)

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
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# MBLink setup
set(MBLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/mblink/)
include_directories(${MBLINK_DIR}/include)
link_directories(${MBLINK_DIR}/lib)

message(STATUS "Using architecture ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "arm" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(GRMBLINK libgrmblink_tx2.a)
else()
  set(GRMBLINK libgrmblink_linux.a)
endif()


# Source files
add_library(robot_driver SHARED
  src/robot_driver.cpp
  src/robot_driver_utils.cpp
  src/controllers/leg_controller.cpp
  src/controllers/inverse_dynamics_controller.cpp
  # src/controllers/underbrush_inverse_dynamics.cpp
  # src/controllers/inertia_estimation_controller.cpp
  # src/controllers/grf_pid_controller.cpp
  src/controllers/joint_controller.cpp
  src/estimators/state_estimator.cpp
  src/estimators/comp_filter_estimator.cpp
  src/estimators/ekf_estimator.cpp
  src/hardware_interfaces/hardware_interface.cpp
  src/hardware_interfaces/spirit_interface.cpp
)

target_link_libraries(robot_driver ${GRMBLINK})
target_include_directories(robot_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_driver
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${quad_msgs_LIBRARIES}
  ${quad_utils_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(robot_driver
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
  Eigen3
  tf2_eigen
  tf2
  geometry_msgs
)

add_executable(robot_driver_node src/robot_driver_node.cpp)
target_link_libraries(robot_driver_node robot_driver)
ament_target_dependencies(robot_driver_node
  rclcpp
  std_msgs
  quad_msgs
  quad_utils
)


install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include
)

install (TARGETS
  robot_driver
  robot_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



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
