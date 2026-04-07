// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/FootPlanDiscrete.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/foot_plan_discrete.h"


#ifndef QUAD_MSGS__MSG__DETAIL__FOOT_PLAN_DISCRETE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__FOOT_PLAN_DISCRETE__STRUCT_H_

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
// Member 'footholds'
#include "quad_msgs/msg/detail/foot_state__struct.h"

/// Struct defined in msg/FootPlanDiscrete in the package quad_msgs.
/**
  * This is a message to hold the discrete foot plan for a single robot foot
  *
  * The plan is defined as a vector of FootState messages
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__FootPlanDiscrete
{
  std_msgs__msg__Header header;
  quad_msgs__msg__FootState__Sequence footholds;
} quad_msgs__msg__FootPlanDiscrete;

// Struct for a sequence of quad_msgs__msg__FootPlanDiscrete.
typedef struct quad_msgs__msg__FootPlanDiscrete__Sequence
{
  quad_msgs__msg__FootPlanDiscrete * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__FootPlanDiscrete__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__FOOT_PLAN_DISCRETE__STRUCT_H_
