// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/BodyState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_state.h"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_STATE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__BODY_STATE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/BodyState in the package quad_msgs.
/**
  * This is a message to hold a robot body state
  *
  * The body state is defined as an pose and twist messages
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__BodyState
{
  std_msgs__msg__Header header;
  uint32_t traj_index;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist twist;
} quad_msgs__msg__BodyState;

// Struct for a sequence of quad_msgs__msg__BodyState.
typedef struct quad_msgs__msg__BodyState__Sequence
{
  quad_msgs__msg__BodyState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__BodyState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_STATE__STRUCT_H_
