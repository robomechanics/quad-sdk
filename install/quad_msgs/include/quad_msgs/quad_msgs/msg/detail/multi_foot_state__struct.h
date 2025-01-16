// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/MultiFootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_state.h"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__STRUCT_H_

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
#include "quad_msgs/msg/detail/foot_state__struct.h"

/// Struct defined in msg/MultiFootState in the package quad_msgs.
/**
  * This is a message to hold the state of all feet of a legged robot
  *
  * The states of each foot are stored in a vector of FootState messages
  * (0 = front left, 1 = back left, 2 = front right, 3 = back right).
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__MultiFootState
{
  std_msgs__msg__Header header;
  uint32_t traj_index;
  quad_msgs__msg__FootState__Sequence feet;
} quad_msgs__msg__MultiFootState;

// Struct for a sequence of quad_msgs__msg__MultiFootState.
typedef struct quad_msgs__msg__MultiFootState__Sequence
{
  quad_msgs__msg__MultiFootState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__MultiFootState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__STRUCT_H_
