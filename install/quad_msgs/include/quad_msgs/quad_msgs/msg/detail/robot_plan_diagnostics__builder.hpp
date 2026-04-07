// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan_diagnostics.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/robot_plan_diagnostics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotPlanDiagnostics_element_times
{
public:
  explicit Init_RobotPlanDiagnostics_element_times(::quad_msgs::msg::RobotPlanDiagnostics & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::RobotPlanDiagnostics element_times(::quad_msgs::msg::RobotPlanDiagnostics::_element_times_type arg)
  {
    msg_.element_times = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlanDiagnostics msg_;
};

class Init_RobotPlanDiagnostics_complexity_schedule
{
public:
  explicit Init_RobotPlanDiagnostics_complexity_schedule(::quad_msgs::msg::RobotPlanDiagnostics & msg)
  : msg_(msg)
  {}
  Init_RobotPlanDiagnostics_element_times complexity_schedule(::quad_msgs::msg::RobotPlanDiagnostics::_complexity_schedule_type arg)
  {
    msg_.complexity_schedule = std::move(arg);
    return Init_RobotPlanDiagnostics_element_times(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlanDiagnostics msg_;
};

class Init_RobotPlanDiagnostics_horizon_length
{
public:
  explicit Init_RobotPlanDiagnostics_horizon_length(::quad_msgs::msg::RobotPlanDiagnostics & msg)
  : msg_(msg)
  {}
  Init_RobotPlanDiagnostics_complexity_schedule horizon_length(::quad_msgs::msg::RobotPlanDiagnostics::_horizon_length_type arg)
  {
    msg_.horizon_length = std::move(arg);
    return Init_RobotPlanDiagnostics_complexity_schedule(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlanDiagnostics msg_;
};

class Init_RobotPlanDiagnostics_iterations
{
public:
  explicit Init_RobotPlanDiagnostics_iterations(::quad_msgs::msg::RobotPlanDiagnostics & msg)
  : msg_(msg)
  {}
  Init_RobotPlanDiagnostics_horizon_length iterations(::quad_msgs::msg::RobotPlanDiagnostics::_iterations_type arg)
  {
    msg_.iterations = std::move(arg);
    return Init_RobotPlanDiagnostics_horizon_length(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlanDiagnostics msg_;
};

class Init_RobotPlanDiagnostics_cost
{
public:
  explicit Init_RobotPlanDiagnostics_cost(::quad_msgs::msg::RobotPlanDiagnostics & msg)
  : msg_(msg)
  {}
  Init_RobotPlanDiagnostics_iterations cost(::quad_msgs::msg::RobotPlanDiagnostics::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return Init_RobotPlanDiagnostics_iterations(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlanDiagnostics msg_;
};

class Init_RobotPlanDiagnostics_compute_time
{
public:
  Init_RobotPlanDiagnostics_compute_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotPlanDiagnostics_cost compute_time(::quad_msgs::msg::RobotPlanDiagnostics::_compute_time_type arg)
  {
    msg_.compute_time = std::move(arg);
    return Init_RobotPlanDiagnostics_cost(msg_);
  }

private:
  ::quad_msgs::msg::RobotPlanDiagnostics msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::RobotPlanDiagnostics>()
{
  return quad_msgs::msg::builder::Init_RobotPlanDiagnostics_compute_time();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__BUILDER_HPP_
