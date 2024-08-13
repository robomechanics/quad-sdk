cmake_minimum_required(VERSION 3.5)
project(quad_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BodyForceEstimate.msg"
  "msg/RobotState.msg"
  "msg/BodyState.msg"
  "msg/GRFArray.msg"
  "msg/RobotPlan.msg"
  "msg/RobotPlanDiagnostics.msg"
  "msg/FootState.msg"
  "msg/MultiFootState.msg"
  "msg/MultiFootPlanContinuous.msg"
  "msg/FootPlanDiscrete.msg"
  "msg/MultiFootPlanDiscrete.msg"
  "msg/LegContactMode.msg"
  "msg/ContactMode.msg"
  "msg/MotorCommand.msg"
  "msg/LegCommand.msg"
  "msg/LegCommandArray.msg"
  DEPENDENCIES std_msgs sensor_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

