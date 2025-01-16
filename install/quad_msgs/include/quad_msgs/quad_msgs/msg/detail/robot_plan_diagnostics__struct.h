// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan_diagnostics.h"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'complexity_schedule'
// Member 'element_times'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RobotPlanDiagnostics in the package quad_msgs.
/**
  * This is a message to hold local plan diagnostics
 */
typedef struct quad_msgs__msg__RobotPlanDiagnostics
{
  double compute_time;
  double cost;
  uint32_t iterations;
  uint32_t horizon_length;
  rosidl_runtime_c__uint32__Sequence complexity_schedule;
  rosidl_runtime_c__double__Sequence element_times;
} quad_msgs__msg__RobotPlanDiagnostics;

// Struct for a sequence of quad_msgs__msg__RobotPlanDiagnostics.
typedef struct quad_msgs__msg__RobotPlanDiagnostics__Sequence
{
  quad_msgs__msg__RobotPlanDiagnostics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__RobotPlanDiagnostics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__STRUCT_H_
