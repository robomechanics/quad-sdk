cmake_minimum_required(VERSION 3.8)
project(quad_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
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
  "msg/BodyForceEstimate.msg"
  "msg/RobotPlanConstraints.msg"
  "msg/FootContact.msg"
  "srv/PlanWithConstraints.srv"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

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
