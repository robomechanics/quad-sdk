#ifndef CMD_VEL_PUBLISHER_H
#define CMD_VEL_PUBLISHER_H

#include "quad_utils/ros_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>

//! A class for publishing random twist velocity commands to the robot
/*!
   CmdVelPublisher is a class for publishing the random twist velocity 
   commands given bounds on x, y, and yaw. Primaily used for testing to 
   assess velocity tracking policy/controller performance
*/
class CmdVelPublisher{
 public:
  /**
  * @brief Constructor for CmdVelPublisher Class
  * @param[in] node ROS Node Shared Pointer to publish and subscribe from
  * @return Construced Object of type CmdVelPublisher
  * 
  */
  CmdVelPublisher(rclcpp::Node::SharedPtr node_);
  /**
   * @brief Calls ros spin_some and pubs data at set frequency
   * 
   */
  void spin();

 private:
  /**
   * @brief Update and publish cmd vel command based on the mode
   * 
   */
  void publishCmdVel();
  /**
   * @brief Sample a new random twist commadn
   * 
   */
  void sampleNewCmd();

  /// Publish the next random velocity command
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  /// Node Pointer to pub and sub from
  rclcpp::Node::SharedPtr node_;

  /// Robot Namespace (e.g. robot_1)
  std::string robot_ns_;

  /// Publish a Single Command or on a Timer
  std::string mode_{"single"};

  /// Update rate for sending and recieving data
  double update_rate_;

  /// Flag to intialize a random vel sample
  bool has_sample_;

  /// Twist resample timer threshold
  double resample_sec_{2.0};

  /// Last velocity command message
  geometry_msgs::msg::Twist last_cmd_vel_msg_;

  /// Time of last velocity command message
  rclcpp::Time last_cmd_vel_msg_time_;

  /// Initialize Random Structs
  std::random_device rd_;
  std::mt19937 rng_;
  std::normal_distribution<double> dist_x_;
  std::normal_distribution<double> dist_y_;
  std::normal_distribution<double> dist_yaw_;

  // Bounds on Twist Commands
  double x_min_, x_max_;
  double y_min_, y_max_;
  double yaw_min_, yaw_max_;

  int seed_;

  double test_duration_;

};
#endif  // CMD_VEL_PUBLIDHER
