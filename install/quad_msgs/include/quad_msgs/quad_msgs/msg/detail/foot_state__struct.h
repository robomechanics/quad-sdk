// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/foot_state.h"


#ifndef QUAD_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_

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
// Member 'position'
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/FootState in the package quad_msgs.
/**
  * This is a message to hold the state of a single foot of a legged robot
  *
  * The states of each foot are stored in a vector of FootState messages
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__FootState
{
  std_msgs__msg__Header header;
  uint32_t traj_index;
  geometry_msgs__msg__Vector3 position;
  geometry_msgs__msg__Vector3 velocity;
  geometry_msgs__msg__Vector3 acceleration;
  bool contact;
} quad_msgs__msg__FootState;

// Struct for a sequence of quad_msgs__msg__FootState.
typedef struct quad_msgs__msg__FootState__Sequence
{
  quad_msgs__msg__FootState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__FootState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_
