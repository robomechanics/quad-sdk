// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/MultiFootPlanDiscrete.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_plan_discrete.h"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_DISCRETE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_DISCRETE__STRUCT_H_

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
// Member 'feet'
#include "quad_msgs/msg/detail/foot_plan_discrete__struct.h"

/// Struct defined in msg/MultiFootPlanDiscrete in the package quad_msgs.
/**
  * This is a message to hold a discrete foot plan for multiple robot feet
  *
  * The plans of each foot are stored in a vector of FootPlanDiscrete messages
  * (0 = front left, 1 = back left, 2 = front right, 3 = back right).
  * Accurate timing information to localize the plans is stored in the header
 */
typedef struct quad_msgs__msg__MultiFootPlanDiscrete
{
  std_msgs__msg__Header header;
  quad_msgs__msg__FootPlanDiscrete__Sequence feet;
} quad_msgs__msg__MultiFootPlanDiscrete;

// Struct for a sequence of quad_msgs__msg__MultiFootPlanDiscrete.
typedef struct quad_msgs__msg__MultiFootPlanDiscrete__Sequence
{
  quad_msgs__msg__MultiFootPlanDiscrete * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__MultiFootPlanDiscrete__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_PLAN_DISCRETE__STRUCT_H_
