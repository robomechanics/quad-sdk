// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/RobotPlanDiagnostics.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan_diagnostics.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__RobotPlanDiagnostics __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__RobotPlanDiagnostics __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotPlanDiagnostics_
{
  using Type = RobotPlanDiagnostics_<ContainerAllocator>;

  explicit RobotPlanDiagnostics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->compute_time = 0.0;
      this->cost = 0.0;
      this->iterations = 0ul;
      this->horizon_length = 0ul;
    }
  }

  explicit RobotPlanDiagnostics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->compute_time = 0.0;
      this->cost = 0.0;
      this->iterations = 0ul;
      this->horizon_length = 0ul;
    }
  }

  // field types and members
  using _compute_time_type =
    double;
  _compute_time_type compute_time;
  using _cost_type =
    double;
  _cost_type cost;
  using _iterations_type =
    uint32_t;
  _iterations_type iterations;
  using _horizon_length_type =
    uint32_t;
  _horizon_length_type horizon_length;
  using _complexity_schedule_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _complexity_schedule_type complexity_schedule;
  using _element_times_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _element_times_type element_times;

  // setters for named parameter idiom
  Type & set__compute_time(
    const double & _arg)
  {
    this->compute_time = _arg;
    return *this;
  }
  Type & set__cost(
    const double & _arg)
  {
    this->cost = _arg;
    return *this;
  }
  Type & set__iterations(
    const uint32_t & _arg)
  {
    this->iterations = _arg;
    return *this;
  }
  Type & set__horizon_length(
    const uint32_t & _arg)
  {
    this->horizon_length = _arg;
    return *this;
  }
  Type & set__complexity_schedule(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->complexity_schedule = _arg;
    return *this;
  }
  Type & set__element_times(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->element_times = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__RobotPlanDiagnostics
    std::shared_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__RobotPlanDiagnostics
    std::shared_ptr<quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotPlanDiagnostics_ & other) const
  {
    if (this->compute_time != other.compute_time) {
      return false;
    }
    if (this->cost != other.cost) {
      return false;
    }
    if (this->iterations != other.iterations) {
      return false;
    }
    if (this->horizon_length != other.horizon_length) {
      return false;
    }
    if (this->complexity_schedule != other.complexity_schedule) {
      return false;
    }
    if (this->element_times != other.element_times) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotPlanDiagnostics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotPlanDiagnostics_

// alias to use template instance with default allocator
using RobotPlanDiagnostics =
  quad_msgs::msg::RobotPlanDiagnostics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN_DIAGNOSTICS__STRUCT_HPP_
