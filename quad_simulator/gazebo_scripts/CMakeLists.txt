cmake_minimum_required(VERSION 3.8)
project(gazebo_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/gz-transport13")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(quad_utils)
find_package(std_msgs)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
  # Mark the existing target as non-global and allow duplicates

# find_library(Protobuf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})

# Link Targets to IGN Transport Libraries
# add_executable(contact_state_publisher_node src/contact_state_publisher_node)
# ament_target_dependencies(contact_state_publisher_node quad_utils std_msgs rclcpp)
# target_link_libraries(contact_state_publisher_node ${IGNITION-TRANSPORT_LIBRARIES})

# Contact State Publisher
add_executable(contact_state_publisher_node
  src/contact_state_publisher_node.cpp
  src/contact_state_publisher.cpp
)
target_include_directories(contact_state_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(contact_state_publisher_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  quad_utils
  ros_gz_interfaces
)

# Install Targets
install(TARGETS contact_state_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(include)

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
