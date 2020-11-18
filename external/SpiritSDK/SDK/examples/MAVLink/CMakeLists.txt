# Copyright (C) Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Written by Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>

# Description: Builds SDK examples Stand and Walk for Spirit or Vision robots.
# See README.md for use.

cmake_minimum_required(VERSION 3.5.0)

set(MBLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../mblink)
set(MBLINK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
set(GRMBLINK _unknown_platform_error)
if(WIN32)
  set(GRMBLINK libgrmblink_win.lib)
elseif(APPLE)
  set(GRMBLINK libgrmblink_mac.a)
elseif(UNIX) # non-APPLE unix assuming Linux 
  set(GRMBLINK libgrmblink_linux.a)
  # Choose this for ARM NVIDIA Xavier instead
  #set(GRMBLINK libgrmblink_xavier.a)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mavlink_examples)

# Find threads library, pthreads on linux/mac
find_package (Threads)

include_directories(${MBLINK_DIR}/include)
link_directories(${MBLINK_LIB_DIR})

# For MAVLink itself
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/eigen)

# Stand
add_executable(stand ${CMAKE_CURRENT_SOURCE_DIR}/stand.cpp)
target_link_libraries(stand ${GRMBLINK} Threads::Threads)

# Walk
add_executable(walk ${CMAKE_CURRENT_SOURCE_DIR}/walk.cpp)
target_link_libraries(walk ${GRMBLINK} Threads::Threads)
