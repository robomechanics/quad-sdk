// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/RobotPlan.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/robot_plan.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'global_plan_timestamp'
// Member 'state_timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'states'
#include "quad_msgs/msg/detail/robot_state__struct.hpp"
// Member 'grfs'
#include "quad_msgs/msg/detail/grf_array__struct.hpp"
// Member 'diagnostics'
#include "quad_msgs/msg/detail/robot_plan_diagnostics__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__RobotPlan __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__RobotPlan __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotPlan_
{
  using Type = RobotPlan_<ContainerAllocator>;

  explicit RobotPlan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    global_plan_timestamp(_init),
    state_timestamp(_init),
    diagnostics(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->compute_time = 0.0;
    }
  }

  explicit RobotPlan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    global_plan_timestamp(_alloc, _init),
    state_timestamp(_alloc, _init),
    diagnostics(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->compute_time = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _global_plan_timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _global_plan_timestamp_type global_plan_timestamp;
  using _state_timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _state_timestamp_type state_timestamp;
  using _states_type =
    std::vector<quad_msgs::msg::RobotState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::RobotState_<ContainerAllocator>>>;
  _states_type states;
  using _grfs_type =
    std::vector<quad_msgs::msg::GRFArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::GRFArray_<ContainerAllocator>>>;
  _grfs_type grfs;
  using _plan_indices_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _plan_indices_type plan_indices;
  using _primitive_ids_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _primitive_ids_type primitive_ids;
  using _compute_time_type =
    double;
  _compute_time_type compute_time;
  using _diagnostics_type =
    quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator>;
  _diagnostics_type diagnostics;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__global_plan_timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->global_plan_timestamp = _arg;
    return *this;
  }
  Type & set__state_timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->state_timestamp = _arg;
    return *this;
  }
  Type & set__states(
    const std::vector<quad_msgs::msg::RobotState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::RobotState_<ContainerAllocator>>> & _arg)
  {
    this->states = _arg;
    return *this;
  }
  Type & set__grfs(
    const std::vector<quad_msgs::msg::GRFArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::GRFArray_<ContainerAllocator>>> & _arg)
  {
    this->grfs = _arg;
    return *this;
  }
  Type & set__plan_indices(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->plan_indices = _arg;
    return *this;
  }
  Type & set__primitive_ids(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->primitive_ids = _arg;
    return *this;
  }
  Type & set__compute_time(
    const double & _arg)
  {
    this->compute_time = _arg;
    return *this;
  }
  Type & set__diagnostics(
    const quad_msgs::msg::RobotPlanDiagnostics_<ContainerAllocator> & _arg)
  {
    this->diagnostics = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::RobotPlan_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::RobotPlan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::RobotPlan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::RobotPlan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__RobotPlan
    std::shared_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__RobotPlan
    std::shared_ptr<quad_msgs::msg::RobotPlan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotPlan_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->global_plan_timestamp != other.global_plan_timestamp) {
      return false;
    }
    if (this->state_timestamp != other.state_timestamp) {
      return false;
    }
    if (this->states != other.states) {
      return false;
    }
    if (this->grfs != other.grfs) {
      return false;
    }
    if (this->plan_indices != other.plan_indices) {
      return false;
    }
    if (this->primitive_ids != other.primitive_ids) {
      return false;
    }
    if (this->compute_time != other.compute_time) {
      return false;
    }
    if (this->diagnostics != other.diagnostics) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotPlan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotPlan_

// alias to use template instance with default allocator
using RobotPlan =
  quad_msgs::msg::RobotPlan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__ROBOT_PLAN__STRUCT_HPP_
