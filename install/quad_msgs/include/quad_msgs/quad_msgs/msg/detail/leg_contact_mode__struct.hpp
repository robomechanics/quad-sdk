// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/LegContactMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_contact_mode.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__STRUCT_HPP_

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
// Member 'contact_forces'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__LegContactMode __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__LegContactMode __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LegContactMode_
{
  using Type = LegContactMode_<ContainerAllocator>;

  explicit LegContactMode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    contact_forces(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->contact_prob = 0.0f;
      this->contact_state = false;
    }
  }

  explicit LegContactMode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    contact_forces(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->contact_prob = 0.0f;
      this->contact_state = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _contact_prob_type =
    float;
  _contact_prob_type contact_prob;
  using _contact_state_type =
    bool;
  _contact_state_type contact_state;
  using _contact_forces_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _contact_forces_type contact_forces;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__contact_prob(
    const float & _arg)
  {
    this->contact_prob = _arg;
    return *this;
  }
  Type & set__contact_state(
    const bool & _arg)
  {
    this->contact_state = _arg;
    return *this;
  }
  Type & set__contact_forces(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->contact_forces = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::LegContactMode_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::LegContactMode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::LegContactMode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::LegContactMode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__LegContactMode
    std::shared_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__LegContactMode
    std::shared_ptr<quad_msgs::msg::LegContactMode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LegContactMode_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->contact_prob != other.contact_prob) {
      return false;
    }
    if (this->contact_state != other.contact_state) {
      return false;
    }
    if (this->contact_forces != other.contact_forces) {
      return false;
    }
    return true;
  }
  bool operator!=(const LegContactMode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LegContactMode_

// alias to use template instance with default allocator
using LegContactMode =
  quad_msgs::msg::LegContactMode_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_CONTACT_MODE__STRUCT_HPP_
