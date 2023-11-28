cmake_minimum_required(VERSION 3.0.2)
<<<<<<< HEAD:open_loop_controller/CMakeLists.txt
project(open_loop_controller)
=======
project(leg_controller)
>>>>>>> Initial commit:leg_controller/CMakeLists.txt

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
<<<<<<< HEAD
  sensor_msgs
  spirit_msgs
  spirit_utils
=======
  quad_msgs
  quad_utils
  eigen_conversions
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
)

catkin_package(
<<<<<<< HEAD
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs spirit_msgs spirit_utils
  DEPENDS 
=======
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp std_msgs quad_msgs quad_utils eigen_conversions
  DEPENDS EIGEN3
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
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

## Declare a C++ library for control
<<<<<<< HEAD:open_loop_controller/CMakeLists.txt
add_library(open_loop_controller
	src/open_loop_controller.cpp
	src/silly_walk_template.cpp
)
add_dependencies(open_loop_controller ${open_loop_controller} ${catkin_EXPORTED_TARGETS})
=======
add_library(leg_controller
	src/leg_controller.cpp
	src/inverse_dynamics.cpp
  src/leg_overrider.cpp
)
add_dependencies(leg_controller ${leg_controller_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
>>>>>>> Initial commit:leg_controller/CMakeLists.txt

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
<<<<<<< HEAD:open_loop_controller/CMakeLists.txt
add_executable(open_loop_controller_node src/open_loop_controller_node.cpp)
add_executable(silly_walk_template_node src/silly_walk_template_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(open_loop_controller_node
  open_loop_controller
	${catkin_LIBRARIES}
)

target_link_libraries(silly_walk_template_node
  open_loop_controller
	${catkin_LIBRARIES}
=======
add_executable(leg_controller_node src/leg_controller_node.cpp)
add_executable(leg_overrider_node src/leg_overrider_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(leg_controller_node
  leg_controller
	${catkin_LIBRARIES}
)
target_link_libraries(leg_overrider_node
  leg_controller
  ${catkin_LIBRARIES}
>>>>>>> Initial commit:leg_controller/CMakeLists.txt
)

#############
## Testing ##
#############

<<<<<<< HEAD:open_loop_controller/CMakeLists.txt
## Google test file for clark_trot_controller library
catkin_add_gtest(open_loop_controller_test test/test_open_loop_controller.cpp)
target_link_libraries(open_loop_controller_test open_loop_controller ${catkin_LIBRARIES})
=======
## Google test file for leg_controller library
catkin_add_gtest(leg_controller_test
  test/test_inverse_dynamics.cpp)
target_link_libraries(leg_controller_test leg_controller ${catkin_LIBRARIES})
>>>>>>> Initial commit:leg_controller/CMakeLists.txt


