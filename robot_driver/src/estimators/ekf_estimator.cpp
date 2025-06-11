#include "robot_driver/estimators/ekf_estimator.hpp"

EKFEstimator::EKFEstimator(rclcpp::Node::SharedPtr node, const std::string& robot_ns): StateEstimator(node, robot_ns) {}

void EKFEstimator::init() {
    RCLCPP_INFO(node_->get_logger(), "EKF_Estimator Initiated");
}

bool EKFEstimator::updateOnce(quad_msgs::msg::RobotState& last_robot_state_msg_) {
    RCLCPP_INFO(node_->get_logger(), "EKF Estimator Updated Once");
    // Test melodic Pipeline
}
