// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/LegCommandArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/leg_command_array.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__STRUCT_HPP_

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
// Member 'leg_commands'
#include "quad_msgs/msg/detail/leg_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__LegCommandArray __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__LegCommandArray __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LegCommandArray_
{
  using Type = LegCommandArray_<ContainerAllocator>;

  explicit LegCommandArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit LegCommandArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _leg_commands_type =
    std::vector<quad_msgs::msg::LegCommand_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::LegCommand_<ContainerAllocator>>>;
  _leg_commands_type leg_commands;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__leg_commands(
    const std::vector<quad_msgs::msg::LegCommand_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::LegCommand_<ContainerAllocator>>> & _arg)
  {
    this->leg_commands = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::LegCommandArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::LegCommandArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::LegCommandArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::LegCommandArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__LegCommandArray
    std::shared_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__LegCommandArray
    std::shared_ptr<quad_msgs::msg::LegCommandArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LegCommandArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->leg_commands != other.leg_commands) {
      return false;
    }
    return true;
  }
  bool operator!=(const LegCommandArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LegCommandArray_

// alias to use template instance with default allocator
using LegCommandArray =
  quad_msgs::msg::LegCommandArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__LEG_COMMAND_ARRAY__STRUCT_HPP_
