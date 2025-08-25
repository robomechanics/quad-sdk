// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/BodyPlan.idl
// generated code does not contain a copyright notice

#ifndef QUAD_MSGS__MSG__DETAIL__BODY_PLAN__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__BODY_PLAN__STRUCT_H_

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
// Member 'plan_indices'
// Member 'primitive_ids'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'states'
#include "nav_msgs/msg/detail/odometry__struct.h"
// Member 'grfs'
#include "quad_msgs/msg/detail/grf_array__struct.h"

/// Struct defined in msg/BodyPlan in the package quad_msgs.
/**
  * This is a message to hold a quad body plan
  *
  * The plan is defined as an array of odometry messages.
  * Accurate timing information for localization is stored in the header.
  * This should match the first state in the states vector.
 */
typedef struct quad_msgs__msg__BodyPlan
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__uint32__Sequence plan_indices;
  rosidl_runtime_c__uint32__Sequence primitive_ids;
  nav_msgs__msg__Odometry__Sequence states;
  quad_msgs__msg__GRFArray__Sequence grfs;
} quad_msgs__msg__BodyPlan;

// Struct for a sequence of quad_msgs__msg__BodyPlan.
typedef struct quad_msgs__msg__BodyPlan__Sequence
{
  quad_msgs__msg__BodyPlan * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__BodyPlan__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_PLAN__STRUCT_H_
