// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/GRFArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/grf_array.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/grf_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'vectors'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'points'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GRFArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vectors
  {
    if (msg.vectors.size() == 0) {
      out << "vectors: []";
    } else {
      out << "vectors: [";
      size_t pending_items = msg.vectors.size();
      for (auto item : msg.vectors) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: contact_states
  {
    if (msg.contact_states.size() == 0) {
      out << "contact_states: []";
    } else {
      out << "contact_states: [";
      size_t pending_items = msg.contact_states.size();
      for (auto item : msg.contact_states) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: traj_index
  {
    out << "traj_index: ";
    rosidl_generator_traits::value_to_yaml(msg.traj_index, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GRFArray & msg,
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

  // member: vectors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vectors.size() == 0) {
      out << "vectors: []\n";
    } else {
      out << "vectors:\n";
      for (auto item : msg.vectors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: contact_states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.contact_states.size() == 0) {
      out << "contact_states: []\n";
    } else {
      out << "contact_states:\n";
      for (auto item : msg.contact_states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GRFArray & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::GRFArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::GRFArray & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::GRFArray>()
{
  return "quad_msgs::msg::GRFArray";
}

template<>
inline const char * name<quad_msgs::msg::GRFArray>()
{
  return "quad_msgs/msg/GRFArray";
}

template<>
struct has_fixed_size<quad_msgs::msg::GRFArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quad_msgs::msg::GRFArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quad_msgs::msg::GRFArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__TRAITS_HPP_
