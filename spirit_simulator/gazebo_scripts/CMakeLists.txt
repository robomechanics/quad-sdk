# MIT License (modified)

# Copyright (c) 2018 The Trustees of the University of Pennsylvania
# Authors:
# Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this **file** (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 2.8.3)
project(gazebo_scripts)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

cmake_policy(SET CMP0054 NEW)
cmake_policy(SET CMP0045 NEW)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    nav_msgs
    std_msgs
    spirit_msgs
    message_filters
    controller_manager
    joint_state_controller
    robot_state_publisher
    tf
    angles
    controller_interface
    control_msgs
    control_toolbox
    forward_command_controller
    realtime_tools
    urdf
    pluginlib
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS
    spirit_msgs
    angles
    controller_interface
    control_msgs
    control_toolbox
    forward_command_controller
    realtime_tools
    urdf
  INCLUDE_DIRS include
  LIBRARIES contact controller
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM include ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Declare a C++ library
add_library(contact SHARED src/contact_plugin.cpp)

add_library(controller SHARED src/controller_plugin.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})

add_library(ground_truth_estimator SHARED src/estimator_plugin.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(odom_publisher src/odom_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(contact ${GAZEBO_LIBRARIES})
target_link_libraries(controller ${GAZEBO_LIBRARIES})
target_link_libraries(odom_publisher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})