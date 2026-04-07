// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quad_msgs:msg/GRFArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quad_msgs/msg/detail/grf_array__rosidl_typesupport_introspection_c.h"
#include "quad_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quad_msgs/msg/detail/grf_array__functions.h"
#include "quad_msgs/msg/detail/grf_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `vectors`
#include "geometry_msgs/msg/vector3.h"
// Member `vectors`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "geometry_msgs/msg/point.h"
// Member `points`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `contact_states`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quad_msgs__msg__GRFArray__init(message_memory);
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_fini_function(void * message_memory)
{
  quad_msgs__msg__GRFArray__fini(message_memory);
}

size_t quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__size_function__GRFArray__vectors(
  const void * untyped_member)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__vectors(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Vector3__Sequence * member =
    (const geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__vectors(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__fetch_function__GRFArray__vectors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Vector3 * item =
    ((const geometry_msgs__msg__Vector3 *)
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__vectors(untyped_member, index));
  geometry_msgs__msg__Vector3 * value =
    (geometry_msgs__msg__Vector3 *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__assign_function__GRFArray__vectors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Vector3 * item =
    ((geometry_msgs__msg__Vector3 *)
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__vectors(untyped_member, index));
  const geometry_msgs__msg__Vector3 * value =
    (const geometry_msgs__msg__Vector3 *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__resize_function__GRFArray__vectors(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Vector3__Sequence * member =
    (geometry_msgs__msg__Vector3__Sequence *)(untyped_member);
  geometry_msgs__msg__Vector3__Sequence__fini(member);
  return geometry_msgs__msg__Vector3__Sequence__init(member, size);
}

size_t quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__size_function__GRFArray__points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__fetch_function__GRFArray__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__points(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__assign_function__GRFArray__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__points(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__resize_function__GRFArray__points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__size_function__GRFArray__contact_states(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__contact_states(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__contact_states(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__fetch_function__GRFArray__contact_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__contact_states(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__assign_function__GRFArray__contact_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__contact_states(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__resize_function__GRFArray__contact_states(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__GRFArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vectors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__GRFArray, vectors),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__size_function__GRFArray__vectors,  // size() function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__vectors,  // get_const(index) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__vectors,  // get(index) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__fetch_function__GRFArray__vectors,  // fetch(index, &value) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__assign_function__GRFArray__vectors,  // assign(index, value) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__resize_function__GRFArray__vectors  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__GRFArray, points),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__size_function__GRFArray__points,  // size() function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__points,  // get_const(index) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__points,  // get(index) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__fetch_function__GRFArray__points,  // fetch(index, &value) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__assign_function__GRFArray__points,  // assign(index, value) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__resize_function__GRFArray__points  // resize(index) function pointer
  },
  {
    "contact_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs__msg__GRFArray, contact_states),  // bytes offset in struct
    NULL,  // default value
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__size_function__GRFArray__contact_states,  // size() function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_const_function__GRFArray__contact_states,  // get_const(index) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__get_function__GRFArray__contact_states,  // get(index) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__fetch_function__GRFArray__contact_states,  // fetch(index, &value) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__assign_function__GRFArray__contact_states,  // assign(index, value) function pointer
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__resize_function__GRFArray__contact_states  // resize(index) function pointer
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
    offsetof(quad_msgs__msg__GRFArray, traj_index),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_members = {
  "quad_msgs__msg",  // message namespace
  "GRFArray",  // message name
  5,  // number of fields
  sizeof(quad_msgs__msg__GRFArray),
  false,  // has_any_key_member_
  quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_member_array,  // message members
  quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_init_function,  // function to initialize message memory (memory has to be allocated)
  quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_type_support_handle = {
  0,
  &quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__GRFArray__get_type_hash,
  &quad_msgs__msg__GRFArray__get_type_description,
  &quad_msgs__msg__GRFArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quad_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quad_msgs, msg, GRFArray)() {
  quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_type_support_handle.typesupport_identifier) {
    quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quad_msgs__msg__GRFArray__rosidl_typesupport_introspection_c__GRFArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
