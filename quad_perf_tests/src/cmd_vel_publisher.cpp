#include "quad_perf_tests/cmd_vel_publisher.hpp"

CmdVelPublisher::CmdVelPublisher(rclcpp::Node::SharedPtr node)
    : node_(node), has_sample_(false) {
  std::string cmd_vel_topic;

  // quad_utils::loadROSParam(node_, "namespace", robot_ns_);
  quad_utils::loadROSParam(node_, "topics.cmd_vel", cmd_vel_topic);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.update_rate",
                           update_rate_);

  quad_utils::loadROSParam(node_, "cmd_vel_publisher.mode", mode_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.resample_sec",
                           resample_sec_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.bounds.x_min", x_min_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.bounds.x_max", x_max_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.bounds.y_min", y_min_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.bounds.y_max", y_max_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.bounds.yaw_min", yaw_min_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.bounds.yaw_max", yaw_max_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.seed", seed_);
  quad_utils::loadROSParam(node_, "cmd_vel_publisher.test_duration",
                           test_duration_);

  cmd_vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  rng_ = std::mt19937(seed_);
  dist_x_ = std::normal_distribution<double>(0.0, (x_max_ - x_min_) / 3.0);
  dist_y_ = std::normal_distribution<double>(0.0, (y_max_ - y_min_) / 3.0);
  dist_yaw_ =
      std::normal_distribution<double>(0.0, (yaw_max_ - yaw_min_) / 3.0);

  // Intialize cmd message to zero command
  last_cmd_vel_msg_ = geometry_msgs::msg::Twist{};
  last_cmd_vel_msg_.linear.x = 0.0;
  last_cmd_vel_msg_.linear.y = 0.0;
  last_cmd_vel_msg_.linear.z = 0.0;
  last_cmd_vel_msg_.angular.x = 0.0;
  last_cmd_vel_msg_.angular.y = 0.0;
  last_cmd_vel_msg_.angular.z = 0.0;
  last_cmd_vel_msg_time_ = node_->now();
}

void CmdVelPublisher::sampleNewCmd() {
  last_cmd_vel_msg_.linear.x = dist_x_(rng_);
  last_cmd_vel_msg_.linear.y = dist_y_(rng_);
  last_cmd_vel_msg_.linear.z = 0.0;  // planar
  last_cmd_vel_msg_.angular.x = 0.0;
  last_cmd_vel_msg_.angular.y = 0.0;
  last_cmd_vel_msg_.angular.z = dist_yaw_(rng_);
  has_sample_ = true;
  last_cmd_vel_msg_time_ = node_->now();
}

void CmdVelPublisher::publishCmdVel() {
  const rclcpp::Time now = node_->now();

  if (mode_ == "off") {
    geometry_msgs::msg::Twist zero{};
    cmd_vel_pub_->publish(zero);
    return;
  } else if (mode_ == "single") {
    if (!has_sample_) {
      sampleNewCmd();  // sample once, then persist
    }
    if ((now - last_cmd_vel_msg_time_).seconds() > test_duration_) {
      last_cmd_vel_msg_.linear.x = 0.0;
      last_cmd_vel_msg_.linear.y = 0.0;
      last_cmd_vel_msg_.linear.z = 0.0;
      last_cmd_vel_msg_.angular.x = 0.0;
      last_cmd_vel_msg_.angular.y = 0.0;
      last_cmd_vel_msg_.angular.z = 0.0;
    }
    cmd_vel_pub_->publish(last_cmd_vel_msg_);
    return;
  } else if (mode_ == "timer") {
    if (!has_sample_) {
      sampleNewCmd();
    } else {
      const double dt = (now - last_cmd_vel_msg_time_).seconds();
      if (dt >= resample_sec_) {
        sampleNewCmd();  // refresh command after threshold
      }
    }
    cmd_vel_pub_->publish(last_cmd_vel_msg_);
    return;
  }
}

void CmdVelPublisher::spin() {
  rclcpp::Rate r(update_rate_);

  while (rclcpp::ok()) {
    publishCmdVel();
    // Collect New Messages
    rclcpp::spin_some(node_);

    // Enforce update rate
    r.sleep();
  }
}
