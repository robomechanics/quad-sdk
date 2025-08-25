// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quad_msgs:msg/BodyPlan.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quad_msgs/msg/detail/body_plan__rosidl_typesupport_introspection_c.h"
#include "quad_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quad_msgs/msg/detail/body_plan__functions.h"
#include "quad_msgs/msg/detail/body_plan__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `plan_indices`
// Member `primitive_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `states`
#include "nav_msgs/msg/odometry.h"
// Member `states`
#include "nav_msgs/msg/detail/odometry__rosidl_typesupport_introspection_c.h"
// Member `grfs`
#include "quad_msgs/msg/grf_array.h"
// Member `grfs`
#include "quad_msgs/msg/detail/grf_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quad_msgs__msg__BodyPlan__init(message_memory);
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_fini_function(void * message_memory)
{
  quad_msgs__msg__BodyPlan__fini(message_memory);
}

size_t quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__plan_indices(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__plan_indices(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__plan_indices(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__plan_indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__plan_indices(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__plan_indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__plan_indices(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__plan_indices(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__primitive_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__primitive_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__primitive_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__primitive_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__primitive_ids(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__primitive_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__primitive_ids(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__primitive_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__states(
  const void * untyped_member)
{
  const nav_msgs__msg__Odometry__Sequence * member =
    (const nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__states(
  const void * untyped_member, size_t index)
{
  const nav_msgs__msg__Odometry__Sequence * member =
    (const nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__states(
  void * untyped_member, size_t index)
{
  nav_msgs__msg__Odometry__Sequence * member =
    (nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const nav_msgs__msg__Odometry * item =
    ((const nav_msgs__msg__Odometry *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__states(untyped_member, index));
  nav_msgs__msg__Odometry * value =
    (nav_msgs__msg__Odometry *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  nav_msgs__msg__Odometry * item =
    ((nav_msgs__msg__Odometry *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__states(untyped_member, index));
  const nav_msgs__msg__Odometry * value =
    (const nav_msgs__msg__Odometry *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__states(
  void * untyped_member, size_t size)
{
  nav_msgs__msg__Odometry__Sequence * member =
    (nav_msgs__msg__Odometry__Sequence *)(untyped_member);
  nav_msgs__msg__Odometry__Sequence__fini(member);
  return nav_msgs__msg__Odometry__Sequence__init(member, size);
}

size_t quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__grfs(
  const void * untyped_member)
{
  const quad_msgs__msg__GRFArray__Sequence * member =
    (const quad_msgs__msg__GRFArray__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__grfs(
  const void * untyped_member, size_t index)
{
  const quad_msgs__msg__GRFArray__Sequence * member =
    (const quad_msgs__msg__GRFArray__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__grfs(
  void * untyped_member, size_t index)
{
  quad_msgs__msg__GRFArray__Sequence * member =
    (quad_msgs__msg__GRFArray__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__grfs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const quad_msgs__msg__GRFArray * item =
    ((const quad_msgs__msg__GRFArray *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__grfs(untyped_member, index));
  quad_msgs__msg__GRFArray * value =
    (quad_msgs__msg__GRFArray *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__grfs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  quad_msgs__msg__GRFArray * item =
    ((quad_msgs__msg__GRFArray *)
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__grfs(untyped_member, index));
  const quad_msgs__msg__GRFArray * value =
    (const quad_msgs__msg__GRFArray *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__grfs(
  void * untyped_member, size_t size)
{
  quad_msgs__msg__GRFArray__Sequence * member =
    (quad_msgs__msg__GRFArray__Sequence *)(untyped_member);
  quad_msgs__msg__GRFArray__Sequence__fini(member);
  return quad_msgs__msg__GRFArray__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__BodyPlan, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "plan_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__BodyPlan, plan_indices),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__plan_indices,  // size() function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__plan_indices,  // get_const(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__plan_indices,  // get(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__plan_indices,  // fetch(index, &value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__plan_indices,  // assign(index, value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__plan_indices  // resize(index) function pointer
  },
  {
    "primitive_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__BodyPlan, primitive_ids),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__primitive_ids,  // size() function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__primitive_ids,  // get_const(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__primitive_ids,  // get(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__primitive_ids,  // fetch(index, &value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__primitive_ids,  // assign(index, value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__primitive_ids  // resize(index) function pointer
  },
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__BodyPlan, states),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__states,  // size() function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__states,  // get_const(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__states,  // get(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__states,  // fetch(index, &value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__states,  // assign(index, value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__states  // resize(index) function pointer
  },
  {
    "grfs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__BodyPlan, grfs),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__size_function__BodyPlan__grfs,  // size() function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_const_function__BodyPlan__grfs,  // get_const(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__get_function__BodyPlan__grfs,  // get(index) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__fetch_function__BodyPlan__grfs,  // fetch(index, &value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__assign_function__BodyPlan__grfs,  // assign(index, value) function pointer
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__resize_function__BodyPlan__grfs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_members = {
  "quad_msgs__msg",  // message namespace
  "BodyPlan",  // message name
  5,  // number of fields
  sizeof(quad_msgs__msg__BodyPlan),
  quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_member_array,  // message members
  quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_init_function,  // function to initialize message memory (memory has to be allocated)
  quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_type_support_handle = {
  0,
  &quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quad_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, BodyPlan)() {
  quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, Odometry)();
  quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, GRFArray)();
  if (!quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_type_support_handle.typesupport_identifier) {
    quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quad_msgs__msg__BodyPlan__rosidl_typesupport_introspection_c__BodyPlan_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
