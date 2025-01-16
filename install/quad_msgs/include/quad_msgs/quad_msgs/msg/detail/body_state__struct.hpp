// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/BodyState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/body_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__BODY_STATE__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__BODY_STATE__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__BodyState __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__BodyState __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BodyState_
{
  using Type = BodyState_<ContainerAllocator>;

  explicit BodyState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->traj_index = 0ul;
    }
  }

  explicit BodyState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->traj_index = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _traj_index_type =
    uint32_t;
  _traj_index_type traj_index;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__traj_index(
    const uint32_t & _arg)
  {
    this->traj_index = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::BodyState_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::BodyState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::BodyState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::BodyState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::BodyState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::BodyState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::BodyState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::BodyState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::BodyState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::BodyState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__BodyState
    std::shared_ptr<quad_msgs::msg::BodyState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__BodyState
    std::shared_ptr<quad_msgs::msg::BodyState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BodyState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->traj_index != other.traj_index) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const BodyState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BodyState_

// alias to use template instance with default allocator
using BodyState =
  quad_msgs::msg::BodyState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__BODY_STATE__STRUCT_HPP_
