#include "robot_driver/estimators/ekf_estimator.h"

EKFEstimator::EKFEstimator() {}

void EKFEstimator::init(ros::NodeHandle& nh) {
    nh_ = nh;
    std::cout << "EKF Estimator Initiated" << std::endl;
}

bool EKFEstimator::updateOnce(quad_msgs::RobotState& last_robot_state_msg) {
    std::cout << "EKF Estimator Updated Once" << std::endl;
    // Added a Comment to Test Pipeline
}
