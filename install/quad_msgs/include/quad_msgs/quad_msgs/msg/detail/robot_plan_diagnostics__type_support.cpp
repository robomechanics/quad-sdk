// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "quad_msgs/msg/detail/robot_plan_diagnostics__functions.h"
#include "quad_msgs/msg/detail/robot_plan_diagnostics__struct.hpp"
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

void RobotPlanDiagnostics_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) quad_msgs::msg::RobotPlanDiagnostics(_init);
}

void RobotPlanDiagnostics_fini_function(void * message_memory)
{
  auto typed_message = static_cast<quad_msgs::msg::RobotPlanDiagnostics *>(message_memory);
  typed_message->~RobotPlanDiagnostics();
}

size_t size_function__RobotPlanDiagnostics__complexity_schedule(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPlanDiagnostics__complexity_schedule(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPlanDiagnostics__complexity_schedule(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPlanDiagnostics__complexity_schedule(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint32_t *>(
    get_const_function__RobotPlanDiagnostics__complexity_schedule(untyped_member, index));
  auto & value = *reinterpret_cast<uint32_t *>(untyped_value);
  value = item;
}

void assign_function__RobotPlanDiagnostics__complexity_schedule(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint32_t *>(
    get_function__RobotPlanDiagnostics__complexity_schedule(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint32_t *>(untyped_value);
  item = value;
}

void resize_function__RobotPlanDiagnostics__complexity_schedule(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__RobotPlanDiagnostics__element_times(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RobotPlanDiagnostics__element_times(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__RobotPlanDiagnostics__element_times(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__RobotPlanDiagnostics__element_times(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RobotPlanDiagnostics__element_times(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RobotPlanDiagnostics__element_times(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RobotPlanDiagnostics__element_times(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__RobotPlanDiagnostics__element_times(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotPlanDiagnostics_message_member_array[6] = {
  {
    "compute_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlanDiagnostics, compute_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cost",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlanDiagnostics, cost),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "iterations",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlanDiagnostics, iterations),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "horizon_length",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlanDiagnostics, horizon_length),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "complexity_schedule",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlanDiagnostics, complexity_schedule),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPlanDiagnostics__complexity_schedule,  // size() function pointer
    get_const_function__RobotPlanDiagnostics__complexity_schedule,  // get_const(index) function pointer
    get_function__RobotPlanDiagnostics__complexity_schedule,  // get(index) function pointer
    fetch_function__RobotPlanDiagnostics__complexity_schedule,  // fetch(index, &value) function pointer
    assign_function__RobotPlanDiagnostics__complexity_schedule,  // assign(index, value) function pointer
    resize_function__RobotPlanDiagnostics__complexity_schedule  // resize(index) function pointer
  },
  {
    "element_times",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quad_msgs::msg::RobotPlanDiagnostics, element_times),  // bytes offset in struct
    nullptr,  // default value
    size_function__RobotPlanDiagnostics__element_times,  // size() function pointer
    get_const_function__RobotPlanDiagnostics__element_times,  // get_const(index) function pointer
    get_function__RobotPlanDiagnostics__element_times,  // get(index) function pointer
    fetch_function__RobotPlanDiagnostics__element_times,  // fetch(index, &value) function pointer
    assign_function__RobotPlanDiagnostics__element_times,  // assign(index, value) function pointer
    resize_function__RobotPlanDiagnostics__element_times  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotPlanDiagnostics_message_members = {
  "quad_msgs::msg",  // message namespace
  "RobotPlanDiagnostics",  // message name
  6,  // number of fields
  sizeof(quad_msgs::msg::RobotPlanDiagnostics),
  false,  // has_any_key_member_
  RobotPlanDiagnostics_message_member_array,  // message members
  RobotPlanDiagnostics_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotPlanDiagnostics_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotPlanDiagnostics_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotPlanDiagnostics_message_members,
  get_message_typesupport_handle_function,
  &quad_msgs__msg__RobotPlanDiagnostics__get_type_hash,
  &quad_msgs__msg__RobotPlanDiagnostics__get_type_description,
  &quad_msgs__msg__RobotPlanDiagnostics__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace quad_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<quad_msgs::msg::RobotPlanDiagnostics>()
{
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::RobotPlanDiagnostics_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, quad_msgs, msg, RobotPlanDiagnostics)() {
  return &::quad_msgs::msg::rosidl_typesupport_introspection_cpp::RobotPlanDiagnostics_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
