cmake_minimum_required(VERSION 3.0.2)
project(global_body_planner)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14 -ffast-math)

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quad_msgs
  quad_utils
  grid_map_core
  grid_map_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES body_planner
 CATKIN_DEPENDS roscpp std_msgs grid_map_core grid_map_ros quad_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# JN Added
set(MY_LIB_SOURCES
  src/global_body_planner.cpp
  src/planning_utils.cpp
  src/global_body_plan.cpp
  src/graph_class.cpp
  src/planner_class.cpp
  src/rrt.cpp
  src/rrt_connect.cpp)

# Declare a C++ library
add_library(${PROJECT_NAME}
  ${MY_LIB_SOURCES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/global_body_planner_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_global_body_planner.cpp
  test/test_kinematics.cpp
  test/test_performance.cpp
  test/test_rotate_grf.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})