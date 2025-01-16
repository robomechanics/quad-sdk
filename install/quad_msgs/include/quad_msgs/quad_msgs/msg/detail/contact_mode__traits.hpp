// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/ContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/contact_mode.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/contact_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'leg_contacts'
#include "quad_msgs/msg/detail/leg_contact_mode__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ContactMode & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: leg_contacts
  {
    if (msg.leg_contacts.size() == 0) {
      out << "leg_contacts: []";
    } else {
      out << "leg_contacts: [";
      size_t pending_items = msg.leg_contacts.size();
      for (auto item : msg.leg_contacts) {
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
  const ContactMode & msg,
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

  // member: leg_contacts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.leg_contacts.size() == 0) {
      out << "leg_contacts: []\n";
    } else {
      out << "leg_contacts:\n";
      for (auto item : msg.leg_contacts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ContactMode & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::ContactMode & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::ContactMode & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::ContactMode>()
{
  return "quad_msgs::msg::ContactMode";
}

template<>
inline const char * name<quad_msgs::msg::ContactMode>()
{
  return "quad_msgs/msg/ContactMode";
}

template<>
struct has_fixed_size<quad_msgs::msg::ContactMode>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quad_msgs::msg::ContactMode>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quad_msgs::msg::ContactMode>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__CONTACT_MODE__TRAITS_HPP_
