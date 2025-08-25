cmake_minimum_required(VERSION 3.5)
project(quad_msgs)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(ALIAS_ROS2_TF2)
  add_definitions(-Dtf2=ros2_tf2)
  add_definitions(-Dtf2_ros=ros2_tf2_ros)
  add_definitions(-Dmessage_filters=ros2_message_filters)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package( builtin_interfaces REQUIRED)
find_package( class_loader REQUIRED)
find_package( console_bridge REQUIRED)
find_package( geometry_msgs REQUIRED)
find_package( nav_msgs REQUIRED)
find_package( rclcpp REQUIRED)
find_package( rcutils REQUIRED)
find_package( sensor_msgs REQUIRED)
find_package( std_msgs REQUIRED)
include_directories("/usr/local/MATLAB/R2024a/extern/include")
include_directories("/usr/local/MATLAB/R2024a/extern/include/MatlabDataArray")
include_directories("/usr/local/MATLAB/R2024a/toolbox/ros/include/ros2")
include_directories("include/quad_msgs")
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(msg_files
  "msg/BodyForceEstimate.msg"
  "msg/BodyPlan.msg"
  "msg/BodyState.msg"
  "msg/ContactMode.msg"
  "msg/FootPlanDiscrete.msg"
  "msg/FootState.msg"
  "msg/GRFArray.msg"
  "msg/LegCommand.msg"
  "msg/LegCommandArray.msg"
  "msg/LegContactMode.msg"
  "msg/LocalPlan.msg"
  "msg/MotorCommand.msg"
  "msg/MultiFootPlanContinuous.msg"
  "msg/MultiFootPlanDiscrete.msg"
  "msg/MultiFootState.msg"
  "msg/RobotPlan.msg"
  "msg/RobotPlanDiagnostics.msg"
  "msg/RobotState.msg"
)
set(srv_files
)
set(action_files
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES 
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)
link_directories("/usr/local/MATLAB/R2024a/extern/bin/glnxa64")
# Library
add_library(
    quad_msgs_matlab 
    SHARED
    src/quad_msgs_BodyForceEstimate_message.cpp
    src/quad_msgs_BodyPlan_message.cpp
    src/quad_msgs_BodyState_message.cpp
    src/quad_msgs_ContactMode_message.cpp
    src/quad_msgs_FootPlanDiscrete_message.cpp
    src/quad_msgs_FootState_message.cpp
    src/quad_msgs_GRFArray_message.cpp
    src/quad_msgs_LegCommand_message.cpp
    src/quad_msgs_LegCommandArray_message.cpp
    src/quad_msgs_LegContactMode_message.cpp
    src/quad_msgs_LocalPlan_message.cpp
    src/quad_msgs_MotorCommand_message.cpp
    src/quad_msgs_MultiFootPlanContinuous_message.cpp
    src/quad_msgs_MultiFootPlanDiscrete_message.cpp
    src/quad_msgs_MultiFootState_message.cpp
    src/quad_msgs_RobotPlan_message.cpp
    src/quad_msgs_RobotPlanDiagnostics_message.cpp
    src/quad_msgs_RobotState_message.cpp
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
target_compile_options(quad_msgs_matlab PUBLIC 
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -Wall -Wextra -Wpedantic
  >)
endif()
target_include_directories(quad_msgs_matlab PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries( quad_msgs_matlab
	"/usr/local/MATLAB/R2024a/sys/os/glnxa64/orig/libstdc++.so.6"
	"/usr/local/MATLAB/R2024a/extern/bin/glnxa64/libMatlabDataArray.so"
	${CMAKE_DL_LIBS}
	)
ament_target_dependencies(
  quad_msgs_matlab
  "builtin_interfaces"
  "class_loader"
  "console_bridge"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(quad_msgs_matlab "${cpp_typesupport_target}")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(quad_msgs_matlab PRIVATE "QUAD_MSGS_BUILDING_LIBRARY")
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS quad_msgs_matlab
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install (DIRECTORY m/ DESTINATION m/ FILES_MATCHING PATTERN "*.m")
ament_export_include_directories(
  include
)
ament_export_libraries(
  quad_msgs_matlab
)
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/_ert_rtw/${file_i}
				   )
add_custom_command(
                   TARGET 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/codegen/exe//${file_i}
				   )				   
endforeach(file_i)
ament_package()
# Generated on Sun Aug 24 22:14:39 2025
