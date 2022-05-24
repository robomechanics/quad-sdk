#include "robot_driver/estimators/ekf_filter_estimator.h"

EKFFilterEstimator::EKFFilterEstimator() {}

void EKFFilterEstimator::init() {
  std::cout << "EKF Estimator Initiated" << std::endl;
}

bool EKFFilterEstimator::updateState() {
  quad_msgs::RobotState state_est;
  std::cout << "EKF Estimator Updated Once" << std::endl;
  return false;
}
