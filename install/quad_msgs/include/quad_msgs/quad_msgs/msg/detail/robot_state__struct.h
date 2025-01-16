// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quad_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_state.h"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

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
// Member 'body'
#include "quad_msgs/msg/detail/body_state__struct.h"
// Member 'joints'
#include "sensor_msgs/msg/detail/joint_state__struct.h"
// Member 'feet'
#include "quad_msgs/msg/detail/multi_foot_state__struct.h"

/// Struct defined in msg/RobotState in the package quad_msgs.
/**
  * This is a message to hold a robot state
  *
  * The state is defined as an Odometry message with the body odometry, and a JointState message with joint positions and velocities
  * Accurate timing information is stored in the header
 */
typedef struct quad_msgs__msg__RobotState
{
  std_msgs__msg__Header header;
  uint32_t traj_index;
  quad_msgs__msg__BodyState body;
  sensor_msgs__msg__JointState joints;
  quad_msgs__msg__MultiFootState feet;
} quad_msgs__msg__RobotState;

// Struct for a sequence of quad_msgs__msg__RobotState.
typedef struct quad_msgs__msg__RobotState__Sequence
{
  quad_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quad_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
