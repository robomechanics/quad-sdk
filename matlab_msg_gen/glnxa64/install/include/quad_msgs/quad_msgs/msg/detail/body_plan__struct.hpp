// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/BodyPlan.idl
// generated code does not contain a copyright notice

#ifndef QUAD_MSGS__MSG__DETAIL__BODY_PLAN__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_PLAN__STRUCT_HPP_

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
// Member 'states'
#include "nav_msgs/msg/detail/odometry__struct.hpp"
// Member 'grfs'
#include "quad_msgs/msg/detail/grf_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__BodyPlan __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__BodyPlan __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BodyPlan_
{
  using Type = BodyPlan_<ContainerAllocator>;

  explicit BodyPlan_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BodyPlan_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _plan_indices_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _plan_indices_type plan_indices;
  using _primitive_ids_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _primitive_ids_type primitive_ids;
  using _states_type =
    std::vector<nav_msgs::msg::Odometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::Odometry_<ContainerAllocator>>>;
  _states_type states;
  using _grfs_type =
    std::vector<quad_msgs::msg::GRFArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::GRFArray_<ContainerAllocator>>>;
  _grfs_type grfs;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
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
  Type & set__states(
    const std::vector<nav_msgs::msg::Odometry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<nav_msgs::msg::Odometry_<ContainerAllocator>>> & _arg)
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

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::BodyPlan_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::BodyPlan_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::BodyPlan_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::BodyPlan_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__BodyPlan
    std::shared_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__BodyPlan
    std::shared_ptr<quad_msgs::msg::BodyPlan_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BodyPlan_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->plan_indices != other.plan_indices) {
      return false;
    }
    if (this->primitive_ids != other.primitive_ids) {
      return false;
    }
    if (this->states != other.states) {
      return false;
    }
    if (this->grfs != other.grfs) {
      return false;
    }
    return true;
  }
  bool operator!=(const BodyPlan_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BodyPlan_

// alias to use template instance with default allocator
using BodyPlan =
  quad_msgs::msg::BodyPlan_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_PLAN__STRUCT_HPP_
