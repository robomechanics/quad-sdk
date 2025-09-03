cmake_minimum_required(VERSION 3.8)
project(splitbelt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz-sim7 QUIET)
if (NOT gz-sim7_FOUND)
  find_package(gz-sim8 QUIET)
endif()

if (gz-sim7_FOUND)
  set(GZ_SIM_VER 7)
elseif (gz-sim8_FOUND)
  set(GZ_SIM_VER 8)
else()
  message(FATAL_ERROR "gz-sim (Gazebo Sim) not found. Install Gazebo Harmonic dev libs.")
endif()

# other dependencies
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

# head files
include_directories(
  include
)

add_library(splitbelt_plugin SHARED
  src/SplitBeltPlugin.cpp
)

target_link_libraries(splitbelt_plugin
  rclcpp::rclcpp
)

ament_target_dependencies(splitbelt_plugin
  rclcpp
  std_msgs
)

# Link Gazebo target
if (GZ_SIM_VER EQUAL 7)
  target_link_libraries(splitbelt_plugin
    gz-sim7::gz-sim7
    gz-math7::gz-math7
    gz-common5::gz-common5
  )
elseif (GZ_SIM_VER EQUAL 8)
  target_link_libraries(splitbelt_plugin
    gz-sim8::gz-sim8
    gz-math7::gz-math7
    gz-common5::gz-common5
  )
endif()

# export plugin
ament_export_libraries(splitbelt_plugin)

install(TARGETS splitbelt_plugin
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
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
