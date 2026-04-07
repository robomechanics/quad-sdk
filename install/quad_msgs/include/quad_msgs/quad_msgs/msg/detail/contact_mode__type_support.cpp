// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from quad_msgs:msg/ContactMode.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/detail/contact_mode__functions.h"
#include "quad_msgs/msg/detail/contact_mode__struct.hpp"
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

void ContactMode_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) quad_msgs::msg::ContactMode(_init);
}

void ContactMode_fini_function(void * message_memory)
{
  auto typed_message = static_cast<quad_msgs::msg::ContactMode *>(message_memory);
  typed_message->~ContactMode();
}

size_t size_function__ContactMode__leg_contacts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<quad_msgs::msg::LegContactMode> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ContactMode__leg_contacts(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<quad_msgs::msg::LegContactMode> *>(untyped_member);
  return &member[index];
}

void * get_function__ContactMode__leg_contacts(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<quad_msgs::msg::LegContactMode> *>(untyped_member);
  return &member[index];
}

void fetch_function__ContactMode__leg_contacts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const quad_msgs::msg::LegContactMode *>(
    get_const_function__ContactMode__leg_contacts(untyped_member, index));
  auto & value = *reinterpret_cast<quad_msgs::msg::LegContactMode *>(untyped_value);
  value = item;
}

void assign_function__ContactMode__leg_contacts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<quad_msgs::msg::LegContactMode *>(
    get_function__ContactMode__leg_contacts(untyped_member, index));
  const auto & value = *reinterpret_cast<const quad_msgs::msg::LegContactMode *>(untyped_value);
  item = value;
}

void resize_function__ContactMode__leg_contacts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<quad_msgs::msg::LegContactMode> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ContactMode_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::ContactMode, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "leg_contacts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<quad_msgs::msg::LegContactMode>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::ContactMode, leg_contacts),  // bytes offset in struct
    nullptr,  // default value
    size_function__ContactMode__leg_contacts,  // size() function pointer
    get_const_function__ContactMode__leg_contacts,  // get_const(index) function pointer
    get_function__ContactMode__leg_contacts,  // get(index) function pointer
    fetch_function__ContactMode__leg_contacts,  // fetch(index, &value) function pointer
    assign_function__ContactMode__leg_contacts,  // assign(index, value) function pointer
    resize_function__ContactMode__leg_contacts  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ContactMode_message_members = {
  "quad_msgs::msg",  // message namespace
  "ContactMode",  // message name
  2,  // number of fields
  sizeof(quad_msgs::msg::ContactMode),
  false,  // has_any_key_member_
  ContactMode_message_member_array,  // message members
  ContactMode_init_function,  // function to initialize message memory (memory has to be allocated)
  ContactMode_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ContactMode_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ContactMode_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__ContactMode__get_type_hash,
  &quad_msgs__msg__ContactMode__get_type_description,
  &quad_msgs__msg__ContactMode__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace quad_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<quad_msgs::msg::ContactMode>()
{
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::ContactMode_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, quad_msgs, msg, ContactMode)() {
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::ContactMode_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
