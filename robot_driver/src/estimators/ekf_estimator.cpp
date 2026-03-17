#include "robot_driver/estimators/ekf_estimator.hpp"

EKFEstimator::EKFEstimator(rclcpp::Node::SharedPtr node, const std::string& robot_ns, 
                          std::shared_ptr<quad_utils::QuadKD2> quadKD) : StateEstimator(node, robot_ns, quadKD) {}

void EKFEstimator::init() {
    RCLCPP_INFO(node_->get_logger(), "EKF_Estimator Initiated");
}

bool EKFEstimator::updateOnce(quad_msgs::msg::RobotState& last_robot_state_msg_) {
    RCLCPP_INFO(node_->get_logger(), "EKF Estimator Updated Once");
    // Test melodic Pipeline
}
