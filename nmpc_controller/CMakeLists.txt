cmake_minimum_required(VERSION 3.0.2)
project(nmpc_controller)

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
  eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs quad_msgs quad_utils grid_map_core grid_map_ros eigen_conversions
  DEPENDS EIGEN3
  LIBRARIES nmpc_controller
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#message(STATUS ${OSQP_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/coin-or
)
link_directories(/usr/local/lib)

## Declare a C++ library for control
add_library(nmpc_controller
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  src/gen/eval_g_spirit.cpp
  src/gen/eval_jac_g_spirit.cpp
  src/gen/eval_hess_g_spirit.cpp
  src/gen/eval_g_a1.cpp
  src/gen/eval_jac_g_a1.cpp
  src/gen/eval_hess_g_a1.cpp
<<<<<<< HEAD
  src/gen/eval_g_new_platform.cpp
  src/gen/eval_jac_g_new_platform.cpp
  src/gen/eval_hess_g_new_platform.cpp
=======
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  src/gen/eval_g_leg_simple.cpp
  src/gen/eval_jac_g_leg_simple.cpp
  src/gen/eval_hess_g_leg_simple.cpp
  src/gen/eval_g_leg_complex.cpp
  src/gen/eval_jac_g_leg_complex.cpp
  src/gen/eval_hess_g_leg_complex.cpp
  src/gen/eval_g_leg_simple_to_complex.cpp
  src/gen/eval_jac_g_leg_simple_to_complex.cpp
  src/gen/eval_hess_g_leg_simple_to_complex.cpp
  src/gen/eval_g_leg_complex_to_simple.cpp
  src/gen/eval_jac_g_leg_complex_to_simple.cpp
  src/gen/eval_hess_g_leg_complex_to_simple.cpp
  src/quad_nlp_utils.cpp
<<<<<<< HEAD
  src/quad_nlp.cpp
  src/quad_nlp_utils.cpp
  src/adaptive_complexity_utils.cpp
=======
  src/quad_nlp.cpp
  src/quad_nlp_utils.cpp
  src/adaptive_complexity_utils.cpp
=======
  src/eval_g_leg.cpp
  src/eval_jac_g_leg.cpp
  src/eval_hess_g_leg.cpp
  src/eval_g_tail.cpp
  src/eval_jac_g_tail.cpp
  src/eval_hess_g_tail.cpp
  src/quad_nlp.cpp
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  src/nmpc_controller.cpp
)
target_link_libraries(nmpc_controller 
  Eigen3::Eigen 
  ipopt 
  ${catkin_LIBRARIES})
add_dependencies(nmpc_controller 
  ${nmpc_controller_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(nmpc_controller-test 
  test/test_nmpc_controller.cpp
  test/test_adaptive_complexity.cpp)
target_link_libraries(nmpc_controller-test nmpc_controller ${catkin_LIBRARIES})
