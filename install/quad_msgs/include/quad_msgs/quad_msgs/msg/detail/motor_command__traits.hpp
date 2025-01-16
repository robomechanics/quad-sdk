// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/motor_command.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: pos_setpoint
  {
    out << "pos_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_setpoint, out);
    out << ", ";
  }

  // member: vel_setpoint
  {
    out << "vel_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_setpoint, out);
    out << ", ";
  }

  // member: kp
  {
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << ", ";
  }

  // member: kd
  {
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << ", ";
  }

  // member: torque_ff
  {
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
    out << ", ";
  }

  // member: pos_component
  {
    out << "pos_component: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_component, out);
    out << ", ";
  }

  // member: vel_component
  {
    out << "vel_component: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_component, out);
    out << ", ";
  }

  // member: fb_component
  {
    out << "fb_component: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_component, out);
    out << ", ";
  }

  // member: effort
  {
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << ", ";
  }

  // member: fb_ratio
  {
    out << "fb_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_ratio, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorCommand & msg,
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

  // member: pos_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_setpoint, out);
    out << "\n";
  }

  // member: vel_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_setpoint, out);
    out << "\n";
  }

  // member: kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kp: ";
    rosidl_generator_traits::value_to_yaml(msg.kp, out);
    out << "\n";
  }

  // member: kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kd: ";
    rosidl_generator_traits::value_to_yaml(msg.kd, out);
    out << "\n";
  }

  // member: torque_ff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_ff: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_ff, out);
    out << "\n";
  }

  // member: pos_component
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_component: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_component, out);
    out << "\n";
  }

  // member: vel_component
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_component: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_component, out);
    out << "\n";
  }

  // member: fb_component
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_component: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_component, out);
    out << "\n";
  }

  // member: effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << "\n";
  }

  // member: fb_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fb_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.fb_ratio, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorCommand & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::MotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::MotorCommand & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::MotorCommand>()
{
  return "quad_msgs::msg::MotorCommand";
}

template<>
inline const char * name<quad_msgs::msg::MotorCommand>()
{
  return "quad_msgs/msg/MotorCommand";
}

template<>
struct has_fixed_size<quad_msgs::msg::MotorCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<quad_msgs::msg::MotorCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<quad_msgs::msg::MotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
