// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/LegCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command_array.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/leg_command_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'leg_commands'
#include "quad_msgs/msg/detail/leg_command__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LegCommandArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: leg_commands
  {
    if (msg.leg_commands.size() == 0) {
      out << "leg_commands: []";
    } else {
      out << "leg_commands: [";
      size_t pending_items = msg.leg_commands.size();
      for (auto item : msg.leg_commands) {
        to_flow_style_yaml(item, out);
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
  const LegCommandArray & msg,
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

  // member: leg_commands
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.leg_commands.size() == 0) {
      out << "leg_commands: []\n";
    } else {
      out << "leg_commands:\n";
      for (auto item : msg.leg_commands) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LegCommandArray & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::LegCommandArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::LegCommandArray & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::LegCommandArray>()
{
  return "quad_msgs::msg::LegCommandArray";
}

template<>
inline const char * name<quad_msgs::msg::LegCommandArray>()
{
  return "quad_msgs/msg/LegCommandArray";
}

template<>
struct has_fixed_size<quad_msgs::msg::LegCommandArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quad_msgs::msg::LegCommandArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quad_msgs::msg::LegCommandArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__TRAITS_HPP_
