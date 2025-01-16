// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/BodyForceEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_force_estimate.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/body_force_estimate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BodyForceEstimate & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: joint_torques
  {
    if (msg.joint_torques.size() == 0) {
      out << "joint_torques: []";
    } else {
      out << "joint_torques: [";
      size_t pending_items = msg.joint_torques.size();
      for (auto item : msg.joint_torques) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BodyForceEstimate & msg,
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

  // member: joint_torques
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_torques.size() == 0) {
      out << "joint_torques: []\n";
    } else {
      out << "joint_torques:\n";
      for (auto item : msg.joint_torques) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BodyForceEstimate & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::BodyForceEstimate & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::BodyForceEstimate & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::BodyForceEstimate>()
{
  return "quad_msgs::msg::BodyForceEstimate";
}

template<>
inline const char * name<quad_msgs::msg::BodyForceEstimate>()
{
  return "quad_msgs/msg/BodyForceEstimate";
}

template<>
struct has_fixed_size<quad_msgs::msg::BodyForceEstimate>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quad_msgs::msg::BodyForceEstimate>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quad_msgs::msg::BodyForceEstimate>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__TRAITS_HPP_
