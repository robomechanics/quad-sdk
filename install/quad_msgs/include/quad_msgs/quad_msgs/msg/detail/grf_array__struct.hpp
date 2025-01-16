// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/GRFArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/grf_array.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__STRUCT_HPP_

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
// Member 'vectors'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__quad_msgs__msg__GRFArray __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__GRFArray __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GRFArray_
{
  using Type = GRFArray_<ContainerAllocator>;

  explicit GRFArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->traj_index = 0ul;
    }
  }

  explicit GRFArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _vectors_type =
    std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>>;
  _vectors_type vectors;
  using _points_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _points_type points;
  using _contact_states_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _contact_states_type contact_states;
  using _traj_index_type =
    uint32_t;
  _traj_index_type traj_index;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vectors(
    const std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>> & _arg)
  {
    this->vectors = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__contact_states(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->contact_states = _arg;
    return *this;
  }
  Type & set__traj_index(
    const uint32_t & _arg)
  {
    this->traj_index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::GRFArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::GRFArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::GRFArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::GRFArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__GRFArray
    std::shared_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__GRFArray
    std::shared_ptr<quad_msgs::msg::GRFArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GRFArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vectors != other.vectors) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->contact_states != other.contact_states) {
      return false;
    }
    if (this->traj_index != other.traj_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const GRFArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GRFArray_

// alias to use template instance with default allocator
using GRFArray =
  quad_msgs::msg::GRFArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__STRUCT_HPP_
