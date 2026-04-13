#ifndef REMOTE_HEARTBEAT_HPP
#define REMOTE_HEARTBEAT_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

//! A class for implementing a remote heartbeat
/*!
   RemoteHeartbeat publishes stamped messages at a fixed rate as a heartbeat
*/
class RemoteHeartbeat {
 public:
  /**
   * @brief Constructor for RemoteHeartbeat Class
   * @param[in] node ROS2 Node to publish and subscribe from
   * @return Constructed object of type RemoteHeartbeat
   */
  RemoteHeartbeat(rclcpp::Node::SharedPtr node);

  /**
   * @brief Spin at the configured update rate, publishing heartbeat messages
   */
  void spin();

 private:
  /**
   * @brief Callback function to handle new robot heartbeat
   * @param[in] msg header containing robot heartbeat
   */
  void robotHeartbeatCallback(const std_msgs::msg::Header::SharedPtr msg);

  /// ROS2 node
  rclcpp::Node::SharedPtr node_;

  /// Subscriber for robot heartbeat messages
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr robot_heartbeat_sub_;

  /// Publisher for remote heartbeat messages
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr remote_heartbeat_pub_;

  /// Update rate for sending and receiving data
  double update_rate_;

  /// Latency threshold on robot messages for warnings (s)
  double robot_latency_threshold_warn_;

  /// Latency threshold on robot messages for error (s)
  double robot_latency_threshold_error_;
};

#endif  // REMOTE_HEARTBEAT_HPP
