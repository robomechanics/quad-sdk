// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/MultiFootPlanContinuous.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_plan_continuous.h"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_CONTINUOUS__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_CONTINUOUS__STRUCT_H_

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
// Member 'states'
#include "quad_msgs/msg/detail/multi_foot_state__struct.h"

/// Struct defined in msg/MultiFootPlanContinuous in the package quad_msgs.
/**
  * This is a message to hold a continuous foot plan for multiple robot feet
  *
  * The plan is defined as a vector of MultiFootState messages
  * Accurate timing information to localize the plans is stored in the header
 */
typedef struct quad_msgs__msg__MultiFootPlanContinuous
{
  std_msgs__msg__Header header;
  quad_msgs__msg__MultiFootState__Sequence states;
} quad_msgs__msg__MultiFootPlanContinuous;

// Struct for a sequence of quad_msgs__msg__MultiFootPlanContinuous.
typedef struct quad_msgs__msg__MultiFootPlanContinuous__Sequence
{
  quad_msgs__msg__MultiFootPlanContinuous * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__MultiFootPlanContinuous__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_CONTINUOUS__STRUCT_H_
