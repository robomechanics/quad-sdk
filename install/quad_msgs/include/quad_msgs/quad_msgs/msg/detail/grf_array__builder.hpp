// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from quad_msgs:msg/GRFArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "quad_msgs/msg/grf_array.hpp"


#ifndef QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__BUILDER_HPP_
#define QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "quad_msgs/msg/detail/grf_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace quad_msgs
{

namespace msg
{

namespace builder
{

class Init_GRFArray_traj_index
{
public:
  explicit Init_GRFArray_traj_index(::quad_msgs::msg::GRFArray & msg)
  : msg_(msg)
  {}
  ::quad_msgs::msg::GRFArray traj_index(::quad_msgs::msg::GRFArray::_traj_index_type arg)
  {
    msg_.traj_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::quad_msgs::msg::GRFArray msg_;
};

class Init_GRFArray_contact_states
{
public:
  explicit Init_GRFArray_contact_states(::quad_msgs::msg::GRFArray & msg)
  : msg_(msg)
  {}
  Init_GRFArray_traj_index contact_states(::quad_msgs::msg::GRFArray::_contact_states_type arg)
  {
    msg_.contact_states = std::move(arg);
    return Init_GRFArray_traj_index(msg_);
  }

private:
  ::quad_msgs::msg::GRFArray msg_;
};

class Init_GRFArray_points
{
public:
  explicit Init_GRFArray_points(::quad_msgs::msg::GRFArray & msg)
  : msg_(msg)
  {}
  Init_GRFArray_contact_states points(::quad_msgs::msg::GRFArray::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_GRFArray_contact_states(msg_);
  }

private:
  ::quad_msgs::msg::GRFArray msg_;
};

class Init_GRFArray_vectors
{
public:
  explicit Init_GRFArray_vectors(::quad_msgs::msg::GRFArray & msg)
  : msg_(msg)
  {}
  Init_GRFArray_points vectors(::quad_msgs::msg::GRFArray::_vectors_type arg)
  {
    msg_.vectors = std::move(arg);
    return Init_GRFArray_points(msg_);
  }

private:
  ::quad_msgs::msg::GRFArray msg_;
};

class Init_GRFArray_header
{
public:
  Init_GRFArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GRFArray_vectors header(::quad_msgs::msg::GRFArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GRFArray_vectors(msg_);
  }

private:
  ::quad_msgs::msg::GRFArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::quad_msgs::msg::GRFArray>()
{
  return quad_msgs::msg::builder::Init_GRFArray_header();
}

}  // namespace quad_msgs

#endif  // QUAD_MSGS__MSG__DETAIL__GRF_ARRAY__BUILDER_HPP_
