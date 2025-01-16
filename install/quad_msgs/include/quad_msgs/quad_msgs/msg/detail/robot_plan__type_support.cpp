// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from quad_msgs:msg/RobotPlan.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/detail/robot_plan__functions.h"
#include "quad_msgs/msg/detail/robot_plan__struct.hpp"
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

void RobotPlan_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) quad_msgs::msg::RobotPlan(_init);
}

void RobotPlan_fini_function(void * message_memory)
{
  auto typed_message = static_cast<quad_msgs::msg::RobotPlan *>(message_memory);
  typed_message->~RobotPlan();
}

size_t size_function__RobotPlan__states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<quad_msgs::msg::RobotState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPlan__states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<quad_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPlan__states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<quad_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPlan__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const quad_msgs::msg::RobotState *>(
    get_const_function__RobotPlan__states(untyped_member, index));
  auto & value = *reinterpret_cast<quad_msgs::msg::RobotState *>(untyped_value);
  value = item;
}

void assign_function__RobotPlan__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<quad_msgs::msg::RobotState *>(
    get_function__RobotPlan__states(untyped_member, index));
  const auto & value = *reinterpret_cast<const quad_msgs::msg::RobotState *>(untyped_value);
  item = value;
}

void resize_function__RobotPlan__states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<quad_msgs::msg::RobotState> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RobotPlan__grfs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<quad_msgs::msg::GRFArray> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPlan__grfs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<quad_msgs::msg::GRFArray> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPlan__grfs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<quad_msgs::msg::GRFArray> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPlan__grfs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const quad_msgs::msg::GRFArray *>(
    get_const_function__RobotPlan__grfs(untyped_member, index));
  auto & value = *reinterpret_cast<quad_msgs::msg::GRFArray *>(untyped_value);
  value = item;
}

void assign_function__RobotPlan__grfs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<quad_msgs::msg::GRFArray *>(
    get_function__RobotPlan__grfs(untyped_member, index));
  const auto & value = *reinterpret_cast<const quad_msgs::msg::GRFArray *>(untyped_value);
  item = value;
}

void resize_function__RobotPlan__grfs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<quad_msgs::msg::GRFArray> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RobotPlan__plan_indices(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPlan__plan_indices(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPlan__plan_indices(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPlan__plan_indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__RobotPlan__plan_indices(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__RobotPlan__plan_indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__RobotPlan__plan_indices(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

void resize_function__RobotPlan__plan_indices(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RobotPlan__primitive_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPlan__primitive_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPlan__primitive_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPlan__primitive_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__RobotPlan__primitive_ids(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__RobotPlan__primitive_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__RobotPlan__primitive_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

void resize_function__RobotPlan__primitive_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotPlan_message_member_array[9] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "global_plan_timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, global_plan_timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state_timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, state_timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<quad_msgs::msg::RobotState>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, states),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPlan__states,  // size() function pointer
    get_const_function__RobotPlan__states,  // get_const(index) function pointer
    get_function__RobotPlan__states,  // get(index) function pointer
    fetch_function__RobotPlan__states,  // fetch(index, &value) function pointer
    assign_function__RobotPlan__states,  // assign(index, value) function pointer
    resize_function__RobotPlan__states  // resize(index) function pointer
  },
  {
    "grfs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<quad_msgs::msg::GRFArray>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, grfs),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPlan__grfs,  // size() function pointer
    get_const_function__RobotPlan__grfs,  // get_const(index) function pointer
    get_function__RobotPlan__grfs,  // get(index) function pointer
    fetch_function__RobotPlan__grfs,  // fetch(index, &value) function pointer
    assign_function__RobotPlan__grfs,  // assign(index, value) function pointer
    resize_function__RobotPlan__grfs  // resize(index) function pointer
  },
  {
    "plan_indices",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, plan_indices),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPlan__plan_indices,  // size() function pointer
    get_const_function__RobotPlan__plan_indices,  // get_const(index) function pointer
    get_function__RobotPlan__plan_indices,  // get(index) function pointer
    fetch_function__RobotPlan__plan_indices,  // fetch(index, &value) function pointer
    assign_function__RobotPlan__plan_indices,  // assign(index, value) function pointer
    resize_function__RobotPlan__plan_indices  // resize(index) function pointer
  },
  {
    "primitive_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, primitive_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPlan__primitive_ids,  // size() function pointer
    get_const_function__RobotPlan__primitive_ids,  // get_const(index) function pointer
    get_function__RobotPlan__primitive_ids,  // get(index) function pointer
    fetch_function__RobotPlan__primitive_ids,  // fetch(index, &value) function pointer
    assign_function__RobotPlan__primitive_ids,  // assign(index, value) function pointer
    resize_function__RobotPlan__primitive_ids  // resize(index) function pointer
  },
  {
    "compute_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, compute_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "diagnostics",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<quad_msgs::msg::RobotPlanDiagnostics>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlan, diagnostics),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotPlan_message_members = {
  "quad_msgs::msg",  // message namespace
  "RobotPlan",  // message name
  9,  // number of fields
  sizeof(quad_msgs::msg::RobotPlan),
  false,  // has_any_key_member_
  RobotPlan_message_member_array,  // message members
  RobotPlan_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotPlan_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotPlan_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotPlan_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__RobotPlan__get_type_hash,
  &quad_msgs__msg__RobotPlan__get_type_description,
  &quad_msgs__msg__RobotPlan__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace quad_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<quad_msgs::msg::RobotPlan>()
{
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::RobotPlan_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, quad_msgs, msg, RobotPlan)() {
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::RobotPlan_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
