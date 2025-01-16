// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/LegContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_contact_mode.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/leg_contact_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'contact_forces'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LegContactMode & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: contact_prob
  {
    out << "contact_prob: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_prob, out);
    out << ", ";
  }

  // member: contact_state
  {
    out << "contact_state: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_state, out);
    out << ", ";
  }

  // member: contact_forces
  {
    out << "contact_forces: ";
    to_flow_style_yaml(msg.contact_forces, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LegContactMode & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: contact_prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_prob: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_prob, out);
    out << "\n";
  }

  // member: contact_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_state: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_state, out);
    out << "\n";
  }

  // member: contact_forces
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_forces:\n";
    to_block_style_yaml(msg.contact_forces, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LegContactMode & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace quad_msgs

namespace rosidl_generator_traits
{

[[deprecated("use quad_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const quad_msgs::msg::LegContactMode & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::LegContactMode & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::LegContactMode>()
{
  return "quad_msgs::msg::LegContactMode";
}

template<>
inline const char * name<quad_msgs::msg::LegContactMode>()
{
  return "quad_msgs/msg/LegContactMode";
}

template<>
struct has_fixed_size<quad_msgs::msg::LegContactMode>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<quad_msgs::msg::LegContactMode>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<quad_msgs::msg::LegContactMode>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__TRAITS_HPP_
