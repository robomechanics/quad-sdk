#pragma once
#include <array>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "quad_msgs/msg/robot_state.hpp"
#include "quad_utils/quad_kd2.hpp"
#include "quad_utils/ros_utils.hpp"

class MujocoEstimator : public rclcpp::Node {
 public:
  MujocoEstimator();

 private:
  // Callbacks
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishState();

  // Math helpers (mirrors Gazebo plugin q_bw rotation)
  std::array<double, 3> rotateWorldToBody(double qw, double qx, double qy,
                                          double qz, double vx, double vy,
                                          double vz);

  std::array<double, 3> rotateBodyToWorld(double qw, double qx, double qy,
                                          double qz, double vx, double vy,
                                          double vz);

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;

  // Publishers
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<quad_msgs::msg::RobotState>::SharedPtr
      state_body_frame_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest messages
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::JointState::SharedPtr latest_joints_;

  // Kinematics (mirrors Gazebo plugin quadKD_)
  std::shared_ptr<quad_utils::QuadKD2> quadKD_;
  rclcpp::Node::SharedPtr kd_node_;  // helper node for QuadKD2 parameter access
  bool urdf_received_ = false;

  // Joint order (quad-sdk convention)
  std::vector<std::string> quadsdk_joint_order_;
};
