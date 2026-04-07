// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from quad_msgs:msg/MultiFootState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/detail/multi_foot_state__functions.h"
#include "quad_msgs/msg/detail/multi_foot_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace quad_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MultiFootState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) quad_msgs::msg::MultiFootState(_init);
}

void MultiFootState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<quad_msgs::msg::MultiFootState *>(message_memory);
  typed_message->~MultiFootState();
}

size_t size_function__MultiFootState__feet(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<quad_msgs::msg::FootState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiFootState__feet(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<quad_msgs::msg::FootState> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiFootState__feet(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<quad_msgs::msg::FootState> *>(untyped_member);
  return &member[index];
}

void fetch_function__MultiFootState__feet(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const quad_msgs::msg::FootState *>(
    get_const_function__MultiFootState__feet(untyped_member, index));
  auto & value = *reinterpret_cast<quad_msgs::msg::FootState *>(untyped_value);
  value = item;
}

void assign_function__MultiFootState__feet(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<quad_msgs::msg::FootState *>(
    get_function__MultiFootState__feet(untyped_member, index));
  const auto & value = *reinterpret_cast<const quad_msgs::msg::FootState *>(untyped_value);
  item = value;
}

void resize_function__MultiFootState__feet(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<quad_msgs::msg::FootState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MultiFootState_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::MultiFootState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "traj_index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::MultiFootState, traj_index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "feet",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<quad_msgs::msg::FootState>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::MultiFootState, feet),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiFootState__feet,  // size() function pointer
    get_const_function__MultiFootState__feet,  // get_const(index) function pointer
    get_function__MultiFootState__feet,  // get(index) function pointer
    fetch_function__MultiFootState__feet,  // fetch(index, &value) function pointer
    assign_function__MultiFootState__feet,  // assign(index, value) function pointer
    resize_function__MultiFootState__feet  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MultiFootState_message_members = {
  "quad_msgs::msg",  // message namespace
  "MultiFootState",  // message name
  3,  // number of fields
  sizeof(quad_msgs::msg::MultiFootState),
  false,  // has_any_key_member_
  MultiFootState_message_member_array,  // message members
  MultiFootState_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiFootState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MultiFootState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MultiFootState_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__MultiFootState__get_type_hash,
  &quad_msgs__msg__MultiFootState__get_type_description,
  &quad_msgs__msg__MultiFootState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace quad_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<quad_msgs::msg::MultiFootState>()
{
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::MultiFootState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, quad_msgs, msg, MultiFootState)() {
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::MultiFootState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
