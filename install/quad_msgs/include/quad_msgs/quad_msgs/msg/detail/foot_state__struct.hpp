// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/foot_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_

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
// Member 'position'
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__FootState __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__FootState __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FootState_
{
  using Type = FootState_<ContainerAllocator>;

  explicit FootState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init),
    acceleration(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->traj_index = 0ul;
      this->contact = false;
    }
  }

  explicit FootState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->traj_index = 0ul;
      this->contact = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _traj_index_type =
    uint32_t;
  _traj_index_type traj_index;
  using _position_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _contact_type =
    bool;
  _contact_type contact;

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
  Type & set__position(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__contact(
    const bool & _arg)
  {
    this->contact = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::FootState_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::FootState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::FootState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::FootState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::FootState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::FootState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::FootState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::FootState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::FootState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::FootState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__FootState
    std::shared_ptr<quad_msgs::msg::FootState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__FootState
    std::shared_ptr<quad_msgs::msg::FootState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FootState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->traj_index != other.traj_index) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->contact != other.contact) {
      return false;
    }
    return true;
  }
  bool operator!=(const FootState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FootState_

// alias to use template instance with default allocator
using FootState =
  quad_msgs::msg::FootState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_
