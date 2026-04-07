// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/GRFArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/grf_array.h"


#ifndef QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__STRUCT_H_

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
// Member 'vectors'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'contact_states'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/GRFArray in the package quad_msgs.
/**
  * This is a message to hold an array of ground reaction forces and their points of application
  *
  * Accurate timing information is stored in the header.
 */
typedef struct quad_msgs__msg__GRFArray
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3__Sequence vectors;
  geometry_msgs__msg__Point__Sequence points;
  rosidl_runtime_c__boolean__Sequence contact_states;
  uint32_t traj_index;
} quad_msgs__msg__GRFArray;

// Struct for a sequence of quad_msgs__msg__GRFArray.
typedef struct quad_msgs__msg__GRFArray__Sequence
{
  quad_msgs__msg__GRFArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__GRFArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__STRUCT_H_
