// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/BodyForceEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_force_estimate.h"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__STRUCT_H_

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
// Member 'joint_torques'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BodyForceEstimate in the package quad_msgs.
/**
  * This is a message to hold a body contact force estimate
  *
  * The body contact force vector holds the estimated external torques acting on each of the 12 joints.
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__BodyForceEstimate
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence joint_torques;
} quad_msgs__msg__BodyForceEstimate;

// Struct for a sequence of quad_msgs__msg__BodyForceEstimate.
typedef struct quad_msgs__msg__BodyForceEstimate__Sequence
{
  quad_msgs__msg__BodyForceEstimate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__BodyForceEstimate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__STRUCT_H_
