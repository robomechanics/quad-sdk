cmake_minimum_required(VERSION 3.0.2)
project(spirit_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotState.msg
  RobotStateTrajectory.msg
  GRFArray.msg
  BodyPlan.msg
  FootState.msg
  MultiFootState.msg
  MultiFootPlanContinuous.msg
  FootPlanDiscrete.msg
  MultiFootPlanDiscrete.msg
  LegContactMode.msg
  BodyForceEstimate.msg
  ContactMode.msg
  MotorCommand.msg
  LegCommand.msg
  LegCommandArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	sensor_msgs
	std_msgs
	nav_msgs
	geometry_msgs
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
   CATKIN_DEPENDS sensor_msgs std_msgs message_runtime
)
