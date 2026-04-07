// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/RobotPlan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan.h"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'global_plan_timestamp'
// Member 'state_timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'states'
#include "quad_msgs/msg/detail/robot_state__struct.h"
// Member 'grfs'
#include "quad_msgs/msg/detail/grf_array__struct.h"
// Member 'plan_indices'
// Member 'primitive_ids'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'diagnostics'
#include "quad_msgs/msg/detail/robot_plan_diagnostics__struct.h"

/// Struct defined in msg/RobotPlan in the package quad_msgs.
/**
  * This is a message to hold a robot plan
  *
  * The plan is defined as an array of odometry messages.
  * Accurate timing information for localization is stored in the header.
  * This should match the first state in the states vector.
 */
typedef struct quad_msgs__msg__RobotPlan
{
  std_msgs__msg__Header header;
  builtin_interfaces__msg__Time global_plan_timestamp;
  builtin_interfaces__msg__Time state_timestamp;
  quad_msgs__msg__RobotState__Sequence states;
  quad_msgs__msg__GRFArray__Sequence grfs;
  rosidl_runtime_c__uint32__Sequence plan_indices;
  rosidl_runtime_c__uint32__Sequence primitive_ids;
  double compute_time;
  quad_msgs__msg__RobotPlanDiagnostics diagnostics;
} quad_msgs__msg__RobotPlan;

// Struct for a sequence of quad_msgs__msg__RobotPlan.
typedef struct quad_msgs__msg__RobotPlan__Sequence
{
  quad_msgs__msg__RobotPlan * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__RobotPlan__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__STRUCT_H_
