// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan_diagnostics.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__TRAITS_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "quad_msgs/msg/detail/robot_plan_diagnostics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace quad_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotPlanDiagnostics & msg,
  std::ostream & out)
{
  out << "{";
  // member: compute_time
  {
    out << "compute_time: ";
    rosidl_generator_traits::value_to_yaml(msg.compute_time, out);
    out << ", ";
  }

  // member: cost
  {
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << ", ";
  }

  // member: iterations
  {
    out << "iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.iterations, out);
    out << ", ";
  }

  // member: horizon_length
  {
    out << "horizon_length: ";
    rosidl_generator_traits::value_to_yaml(msg.horizon_length, out);
    out << ", ";
  }

  // member: complexity_schedule
  {
    if (msg.complexity_schedule.size() == 0) {
      out << "complexity_schedule: []";
    } else {
      out << "complexity_schedule: [";
      size_t pending_items = msg.complexity_schedule.size();
      for (auto item : msg.complexity_schedule) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: element_times
  {
    if (msg.element_times.size() == 0) {
      out << "element_times: []";
    } else {
      out << "element_times: [";
      size_t pending_items = msg.element_times.size();
      for (auto item : msg.element_times) {
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
  const RobotPlanDiagnostics & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: compute_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compute_time: ";
    rosidl_generator_traits::value_to_yaml(msg.compute_time, out);
    out << "\n";
  }

  // member: cost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << "\n";
  }

  // member: iterations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.iterations, out);
    out << "\n";
  }

  // member: horizon_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "horizon_length: ";
    rosidl_generator_traits::value_to_yaml(msg.horizon_length, out);
    out << "\n";
  }

  // member: complexity_schedule
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.complexity_schedule.size() == 0) {
      out << "complexity_schedule: []\n";
    } else {
      out << "complexity_schedule:\n";
      for (auto item : msg.complexity_schedule) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: element_times
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.element_times.size() == 0) {
      out << "element_times: []\n";
    } else {
      out << "element_times:\n";
      for (auto item : msg.element_times) {
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

inline std::string to_yaml(const RobotPlanDiagnostics & msg, bool use_flow_style = false)
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
  const quad_msgs::msg::RobotPlanDiagnostics & msg,
  std::ostream & out, size_t indentation = 0)
{
  quad_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use quad_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const quad_msgs::msg::RobotPlanDiagnostics & msg)
{
  return quad_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<quad_msgs::msg::RobotPlanDiagnostics>()
{
  return "quad_msgs::msg::RobotPlanDiagnostics";
}

template<>
inline const char * name<quad_msgs::msg::RobotPlanDiagnostics>()
{
  return "quad_msgs/msg/RobotPlanDiagnostics";
}

template<>
struct has_fixed_size<quad_msgs::msg::RobotPlanDiagnostics>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<quad_msgs::msg::RobotPlanDiagnostics>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<quad_msgs::msg::RobotPlanDiagnostics>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__TRAITS_HPP_
