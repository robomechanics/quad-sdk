// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quad_msgs:msg/MotorCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/motor_command.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__quad_msgs__msg__MotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__quad_msgs__msg__MotorCommand __declspec(deprecated)
#endif

namespace quad_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_
{
  using Type = MotorCommand_<ContainerAllocator>;

  explicit MotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_setpoint = 0.0;
      this->vel_setpoint = 0.0;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->torque_ff = 0.0;
      this->pos_component = 0.0;
      this->vel_component = 0.0;
      this->fb_component = 0.0;
      this->effort = 0.0;
      this->fb_ratio = 0.0;
    }
  }

  explicit MotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_setpoint = 0.0;
      this->vel_setpoint = 0.0;
      this->kp = 0.0f;
      this->kd = 0.0f;
      this->torque_ff = 0.0;
      this->pos_component = 0.0;
      this->vel_component = 0.0;
      this->fb_component = 0.0;
      this->effort = 0.0;
      this->fb_ratio = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pos_setpoint_type =
    double;
  _pos_setpoint_type pos_setpoint;
  using _vel_setpoint_type =
    double;
  _vel_setpoint_type vel_setpoint;
  using _kp_type =
    float;
  _kp_type kp;
  using _kd_type =
    float;
  _kd_type kd;
  using _torque_ff_type =
    double;
  _torque_ff_type torque_ff;
  using _pos_component_type =
    double;
  _pos_component_type pos_component;
  using _vel_component_type =
    double;
  _vel_component_type vel_component;
  using _fb_component_type =
    double;
  _fb_component_type fb_component;
  using _effort_type =
    double;
  _effort_type effort;
  using _fb_ratio_type =
    double;
  _fb_ratio_type fb_ratio;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pos_setpoint(
    const double & _arg)
  {
    this->pos_setpoint = _arg;
    return *this;
  }
  Type & set__vel_setpoint(
    const double & _arg)
  {
    this->vel_setpoint = _arg;
    return *this;
  }
  Type & set__kp(
    const float & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const float & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__torque_ff(
    const double & _arg)
  {
    this->torque_ff = _arg;
    return *this;
  }
  Type & set__pos_component(
    const double & _arg)
  {
    this->pos_component = _arg;
    return *this;
  }
  Type & set__vel_component(
    const double & _arg)
  {
    this->vel_component = _arg;
    return *this;
  }
  Type & set__fb_component(
    const double & _arg)
  {
    this->fb_component = _arg;
    return *this;
  }
  Type & set__effort(
    const double & _arg)
  {
    this->effort = _arg;
    return *this;
  }
  Type & set__fb_ratio(
    const double & _arg)
  {
    this->fb_ratio = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quad_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const quad_msgs::msg::MotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quad_msgs::msg::MotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quad_msgs__msg__MotorCommand
    std::shared_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quad_msgs__msg__MotorCommand
    std::shared_ptr<quad_msgs::msg::MotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pos_setpoint != other.pos_setpoint) {
      return false;
    }
    if (this->vel_setpoint != other.vel_setpoint) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->torque_ff != other.torque_ff) {
      return false;
    }
    if (this->pos_component != other.pos_component) {
      return false;
    }
    if (this->vel_component != other.vel_component) {
      return false;
    }
    if (this->fb_component != other.fb_component) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    if (this->fb_ratio != other.fb_ratio) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_

// alias to use template instance with default allocator
using MotorCommand =
  quad_msgs::msg::MotorCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
