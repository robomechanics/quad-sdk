// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/MultiFootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/multi_foot_state.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__STRUCT_HPP_

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
// Member 'feet'
#include "quad_msgs/msg/detail/foot_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__MultiFootState __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__MultiFootState __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiFootState_
{
  using Type = MultiFootState_<ContainerAllocator>;

  explicit MultiFootState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->traj_index = 0ul;
    }
  }

  explicit MultiFootState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
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
  using _feet_type =
    std::vector<quad_msgs::msg::FootState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::FootState_<ContainerAllocator>>>;
  _feet_type feet;

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
  Type & set__feet(
    const std::vector<quad_msgs::msg::FootState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<quad_msgs::msg::FootState_<ContainerAllocator>>> & _arg)
  {
    this->feet = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::MultiFootState_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::MultiFootState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::MultiFootState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::MultiFootState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__MultiFootState
    std::shared_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__MultiFootState
    std::shared_ptr<quad_msgs::msg::MultiFootState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiFootState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->traj_index != other.traj_index) {
      return false;
    }
    if (this->feet != other.feet) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiFootState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiFootState_

// alias to use template instance with default allocator
using MultiFootState =
  quad_msgs::msg::MultiFootState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__MULTI_FOOT_STATE__STRUCT_HPP_
