// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/LegContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_contact_mode.h"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__STRUCT_H_

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
// Member 'contact_forces'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/LegContactMode in the package quad_msgs.
/**
  * This is a message to hold contact mode of one leg
 */
typedef struct quad_msgs__msg__LegContactMode
{
  std_msgs__msg__Header header;
  float contact_prob;
  bool contact_state;
  /// fx,fy,fz
  geometry_msgs__msg__Vector3 contact_forces;
} quad_msgs__msg__LegContactMode;

// Struct for a sequence of quad_msgs__msg__LegContactMode.
typedef struct quad_msgs__msg__LegContactMode__Sequence
{
  quad_msgs__msg__LegContactMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__LegContactMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__STRUCT_H_
