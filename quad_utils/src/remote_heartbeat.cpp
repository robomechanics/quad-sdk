#include "quad_utils/remote_heartbeat.hpp"

#include "quad_utils/ros_utils.hpp"

RemoteHeartbeat::RemoteHeartbeat(rclcpp::Node::SharedPtr node) : node_(node) {
  // Load parameters
  std::string remote_heartbeat_topic, robot_heartbeat_topic;
  quad_utils::loadROSParam(node_, "topics.heartbeat.remote",
                           remote_heartbeat_topic);
  quad_utils::loadROSParam(node_, "topics.heartbeat.robot",
                           robot_heartbeat_topic);
  quad_utils::loadROSParam(node_,
                           "remote_heartbeat.robot_latency_threshold_warn",
                           robot_latency_threshold_warn_);
  quad_utils::loadROSParam(node_,
                           "remote_heartbeat.robot_latency_threshold_error",
                           robot_latency_threshold_error_);
  quad_utils::loadROSParam(node_, "remote_heartbeat.update_rate", update_rate_);

  // Setup pub/sub
  remote_heartbeat_pub_ =
      node_->create_publisher<std_msgs::msg::Header>(remote_heartbeat_topic, 1);
  robot_heartbeat_sub_ = node_->create_subscription<std_msgs::msg::Header>(
      robot_heartbeat_topic, 1,
      std::bind(&RemoteHeartbeat::robotHeartbeatCallback, this,
                std::placeholders::_1));
}

void RemoteHeartbeat::robotHeartbeatCallback(
    const std_msgs::msg::Header::SharedPtr msg) {
  // Get the current time and compare to the message time
  rclcpp::Time sent_time(msg->stamp);
  double last_robot_heartbeat_time = sent_time.seconds();
  double t_now = node_->now().seconds();
  double t_latency = t_now - last_robot_heartbeat_time;

  if (abs(t_latency) >= robot_latency_threshold_warn_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Robot latency = %6.4fs which exceeds the warning "
                         "threshold of %6.4fs",
                         t_latency, robot_latency_threshold_warn_);
  }

  if (abs(t_latency) >= robot_latency_threshold_error_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Robot latency = %6.4fs which exceeds the maximum "
                          "threshold of %6.4fs",
                          t_latency, robot_latency_threshold_error_);
  }
}

void RemoteHeartbeat::spin() {
  rclcpp::Rate r(update_rate_);
  while (rclcpp::ok()) {
    std_msgs::msg::Header msg;
    msg.stamp = node_->now();
    remote_heartbeat_pub_->publish(msg);

    rclcpp::spin_some(node_);
    r.sleep();
  }
}
