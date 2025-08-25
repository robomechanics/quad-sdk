// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/BodyPlan.idl
// generated code does not contain a copyright notice

#ifndef QUAD_MSGS__MSG__DETAIL__BODY_PLAN__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_PLAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/body_plan__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'states'
#include "nav_msgs/msg/detail/odometry__traits.hpp"
// Member 'grfs'
#include "quad_msgs/msg/detail/grf_array__traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BodyPlan & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: plan_indices
  {
    if (msg.plan_indices.size() == 0) {
      out << "plan_indices: []";
    } else {
      out << "plan_indices: [";
      size_t pending_items = msg.plan_indices.size();
      for (auto item : msg.plan_indices) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: primitive_ids
  {
    if (msg.primitive_ids.size() == 0) {
      out << "primitive_ids: []";
    } else {
      out << "primitive_ids: [";
      size_t pending_items = msg.primitive_ids.size();
      for (auto item : msg.primitive_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: states
  {
    if (msg.states.size() == 0) {
      out << "states: []";
    } else {
      out << "states: [";
      size_t pending_items = msg.states.size();
      for (auto item : msg.states) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: grfs
  {
    if (msg.grfs.size() == 0) {
      out << "grfs: []";
    } else {
      out << "grfs: [";
      size_t pending_items = msg.grfs.size();
      for (auto item : msg.grfs) {
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
  const BodyPlan & msg,
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

  // member: plan_indices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.plan_indices.size() == 0) {
      out << "plan_indices: []\n";
    } else {
      out << "plan_indices:\n";
      for (auto item : msg.plan_indices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: primitive_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.primitive_ids.size() == 0) {
      out << "primitive_ids: []\n";
    } else {
      out << "primitive_ids:\n";
      for (auto item : msg.primitive_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: states
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.states.size() == 0) {
      out << "states: []\n";
    } else {
      out << "states:\n";
      for (auto item : msg.states) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: grfs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.grfs.size() == 0) {
      out << "grfs: []\n";
    } else {
      out << "grfs:\n";
      for (auto item : msg.grfs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BodyPlan & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::BodyPlan & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::BodyPlan & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::BodyPlan>()
{
  return "quad_msgs::msg::BodyPlan";
}

template<>
inline const char * name<quad_msgs::msg::BodyPlan>()
{
  return "quad_msgs/msg/BodyPlan";
}

template<>
struct has_fixed_size<quad_msgs::msg::BodyPlan>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quad_msgs::msg::BodyPlan>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quad_msgs::msg::BodyPlan>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_PLAN__TRAITS_HPP_
