// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from quad_msgs:msg/BodyForceEstimate.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/detail/body_force_estimate__functions.h"
#include "quad_msgs/msg/detail/body_force_estimate__struct.hpp"
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

void BodyForceEstimate_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) quad_msgs::msg::BodyForceEstimate(_init);
}

void BodyForceEstimate_fini_function(void * message_memory)
{
  auto typed_message = static_cast<quad_msgs::msg::BodyForceEstimate *>(message_memory);
  typed_message->~BodyForceEstimate();
}

size_t size_function__BodyForceEstimate__joint_torques(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BodyForceEstimate__joint_torques(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__BodyForceEstimate__joint_torques(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__BodyForceEstimate__joint_torques(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BodyForceEstimate__joint_torques(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BodyForceEstimate__joint_torques(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BodyForceEstimate__joint_torques(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__BodyForceEstimate__joint_torques(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BodyForceEstimate_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::BodyForceEstimate, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_torques",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::BodyForceEstimate, joint_torques),  // bytes offset in struct
    nullptr,  // default value
    size_function__BodyForceEstimate__joint_torques,  // size() function pointer
    get_const_function__BodyForceEstimate__joint_torques,  // get_const(index) function pointer
    get_function__BodyForceEstimate__joint_torques,  // get(index) function pointer
    fetch_function__BodyForceEstimate__joint_torques,  // fetch(index, &value) function pointer
    assign_function__BodyForceEstimate__joint_torques,  // assign(index, value) function pointer
    resize_function__BodyForceEstimate__joint_torques  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BodyForceEstimate_message_members = {
  "quad_msgs::msg",  // message namespace
  "BodyForceEstimate",  // message name
  2,  // number of fields
  sizeof(quad_msgs::msg::BodyForceEstimate),
  false,  // has_any_key_member_
  BodyForceEstimate_message_member_array,  // message members
  BodyForceEstimate_init_function,  // function to initialize message memory (memory has to be allocated)
  BodyForceEstimate_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BodyForceEstimate_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BodyForceEstimate_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__BodyForceEstimate__get_type_hash,
  &quad_msgs__msg__BodyForceEstimate__get_type_description,
  &quad_msgs__msg__BodyForceEstimate__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace quad_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<quad_msgs::msg::BodyForceEstimate>()
{
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::BodyForceEstimate_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, quad_msgs, msg, BodyForceEstimate)() {
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::BodyForceEstimate_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
