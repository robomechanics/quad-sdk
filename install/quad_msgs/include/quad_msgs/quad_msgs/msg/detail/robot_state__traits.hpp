// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'body'
#include "quad_msgs/msg/detail/body_state__traits.hpp"
// Member 'joints'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"
// Member 'feet'
#include "quad_msgs/msg/detail/multi_foot_state__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: traj_index
  {
    out << "traj_index: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_index, out);
    out << ", ";
  }

  // member: body
  {
    out << "body: ";
    to_flow_style_yaml(msg.body, out);
    out << ", ";
  }

  // member: joints
  {
    out << "joints: ";
    to_flow_style_yaml(msg.joints, out);
    out << ", ";
  }

  // member: feet
  {
    out << "feet: ";
    to_flow_style_yaml(msg.feet, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
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

  // member: traj_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traj_index: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_index, out);
    out << "\n";
  }

  // member: body
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body:\n";
    to_block_style_yaml(msg.body, out, indentation + 2);
  }

  // member: joints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joints:\n";
    to_block_style_yaml(msg.joints, out, indentation + 2);
  }

  // member: feet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feet:\n";
    to_block_style_yaml(msg.feet, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::RobotState & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::RobotState>()
{
  return "quad_msgs::msg::RobotState";
}

template<>
inline const char * name<quad_msgs::msg::RobotState>()
{
  return "quad_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<quad_msgs::msg::RobotState>
  : std::integral_constant<bool, has_fixed_size<quad_msgs::msg::BodyState>::value && has_fixed_size<quad_msgs::msg::MultiFootState>::value && has_fixed_size<sensor_msgs::msg::JointState>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<quad_msgs::msg::RobotState>
  : std::integral_constant<bool, has_bounded_size<quad_msgs::msg::BodyState>::value && has_bounded_size<quad_msgs::msg::MultiFootState>::value && has_bounded_size<sensor_msgs::msg::JointState>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<quad_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
