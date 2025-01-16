// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quad_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quad_msgs/msg/detail/foot_state__rosidl_typesupport_introspection_c.h"
#include "quad_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quad_msgs/msg/detail/foot_state__functions.h"
#include "quad_msgs/msg/detail/foot_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/vector3.h"
// Member `position`
// Member `velocity`
// Member `acceleration`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quad_msgs__msg__FootState__init(message_memory);
}

void quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_fini_function(void * message_memory)
{
  quad_msgs__msg__FootState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__FootState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "traj_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__FootState, traj_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__FootState, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__FootState, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__FootState, acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "contact",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__FootState, contact),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_members = {
  "quad_msgs__msg",  // message namespace
  "FootState",  // message name
  6,  // number of fields
  sizeof(quad_msgs__msg__FootState),
  false,  // has_any_key_member_
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_member_array,  // message members
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_init_function,  // function to initialize message memory (memory has to be allocated)
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_type_support_handle = {
  0,
  &quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__FootState__get_type_hash,
  &quad_msgs__msg__FootState__get_type_description,
  &quad_msgs__msg__FootState__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quad_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, FootState)() {
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_type_support_handle.typesupport_identifier) {
    quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quad_msgs__msg__FootState__rosidl_typesupport_introspection_c__FootState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
