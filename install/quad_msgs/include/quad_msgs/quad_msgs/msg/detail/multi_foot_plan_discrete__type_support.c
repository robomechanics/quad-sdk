// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quad_msgs:msg/MultiFootPlanDiscrete.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quad_msgs/msg/detail/multi_foot_plan_discrete__rosidl_typesupport_introspection_c.h"
#include "quad_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quad_msgs/msg/detail/multi_foot_plan_discrete__functions.h"
#include "quad_msgs/msg/detail/multi_foot_plan_discrete__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `feet`
#include "quad_msgs/msg/foot_plan_discrete.h"
// Member `feet`
#include "quad_msgs/msg/detail/foot_plan_discrete__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quad_msgs__msg__MultiFootPlanDiscrete__init(message_memory);
}

void quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_fini_function(void * message_memory)
{
  quad_msgs__msg__MultiFootPlanDiscrete__fini(message_memory);
}

size_t quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__size_function__MultiFootPlanDiscrete__feet(
  const void * untyped_member)
{
  const quad_msgs__msg__FootPlanDiscrete__Sequence * member =
    (const quad_msgs__msg__FootPlanDiscrete__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__get_const_function__MultiFootPlanDiscrete__feet(
  const void * untyped_member, size_t index)
{
  const quad_msgs__msg__FootPlanDiscrete__Sequence * member =
    (const quad_msgs__msg__FootPlanDiscrete__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__get_function__MultiFootPlanDiscrete__feet(
  void * untyped_member, size_t index)
{
  quad_msgs__msg__FootPlanDiscrete__Sequence * member =
    (quad_msgs__msg__FootPlanDiscrete__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__fetch_function__MultiFootPlanDiscrete__feet(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const quad_msgs__msg__FootPlanDiscrete * item =
    ((const quad_msgs__msg__FootPlanDiscrete *)
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__get_const_function__MultiFootPlanDiscrete__feet(untyped_member, index));
  quad_msgs__msg__FootPlanDiscrete * value =
    (quad_msgs__msg__FootPlanDiscrete *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__assign_function__MultiFootPlanDiscrete__feet(
  void * untyped_member, size_t index, const void * untyped_value)
{
  quad_msgs__msg__FootPlanDiscrete * item =
    ((quad_msgs__msg__FootPlanDiscrete *)
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__get_function__MultiFootPlanDiscrete__feet(untyped_member, index));
  const quad_msgs__msg__FootPlanDiscrete * value =
    (const quad_msgs__msg__FootPlanDiscrete *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__resize_function__MultiFootPlanDiscrete__feet(
  void * untyped_member, size_t size)
{
  quad_msgs__msg__FootPlanDiscrete__Sequence * member =
    (quad_msgs__msg__FootPlanDiscrete__Sequence *)(untyped_member);
  quad_msgs__msg__FootPlanDiscrete__Sequence__fini(member);
  return quad_msgs__msg__FootPlanDiscrete__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__MultiFootPlanDiscrete, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feet",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__MultiFootPlanDiscrete, feet),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__size_function__MultiFootPlanDiscrete__feet,  // size() function pointer
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__get_const_function__MultiFootPlanDiscrete__feet,  // get_const(index) function pointer
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__get_function__MultiFootPlanDiscrete__feet,  // get(index) function pointer
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__fetch_function__MultiFootPlanDiscrete__feet,  // fetch(index, &value) function pointer
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__assign_function__MultiFootPlanDiscrete__feet,  // assign(index, value) function pointer
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__resize_function__MultiFootPlanDiscrete__feet  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_members = {
  "quad_msgs__msg",  // message namespace
  "MultiFootPlanDiscrete",  // message name
  2,  // number of fields
  sizeof(quad_msgs__msg__MultiFootPlanDiscrete),
  false,  // has_any_key_member_
  quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_member_array,  // message members
  quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_init_function,  // function to initialize message memory (memory has to be allocated)
  quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_type_support_handle = {
  0,
  &quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__MultiFootPlanDiscrete__get_type_hash,
  &quad_msgs__msg__MultiFootPlanDiscrete__get_type_description,
  &quad_msgs__msg__MultiFootPlanDiscrete__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quad_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, MultiFootPlanDiscrete)() {
  quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, FootPlanDiscrete)();
  if (!quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_type_support_handle.typesupport_identifier) {
    quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quad_msgs__msg__MultiFootPlanDiscrete__rosidl_typesupport_introspection_c__MultiFootPlanDiscrete_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
