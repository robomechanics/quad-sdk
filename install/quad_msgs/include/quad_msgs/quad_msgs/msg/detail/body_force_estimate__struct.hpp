// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/BodyForceEstimate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_force_estimate.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__BodyForceEstimate __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__BodyForceEstimate __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BodyForceEstimate_
{
  using Type = BodyForceEstimate_<ContainerAllocator>;

  explicit BodyForceEstimate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit BodyForceEstimate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_torques_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_torques_type joint_torques;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_torques(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_torques = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::BodyForceEstimate_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::BodyForceEstimate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::BodyForceEstimate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::BodyForceEstimate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__BodyForceEstimate
    std::shared_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__BodyForceEstimate
    std::shared_ptr<quad_msgs::msg::BodyForceEstimate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BodyForceEstimate_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_torques != other.joint_torques) {
      return false;
    }
    return true;
  }
  bool operator!=(const BodyForceEstimate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BodyForceEstimate_

// alias to use template instance with default allocator
using BodyForceEstimate =
  quad_msgs::msg::BodyForceEstimate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_FORCE_ESTIMATE__STRUCT_HPP_
