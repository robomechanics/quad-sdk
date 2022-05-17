#include "robot_driver/estimators/state_estimator.h"

StateEstimator::StateEstimator() {
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

void StateEstimator::readIMU(const sensor_msgs::Imu::ConstPtr& last_imu_msg_,
                           Eigen::VectorXd& fk, Eigen::VectorXd& wk,
                           Eigen::Quaterniond& qk) {
  if (last_imu_msg_ != NULL) {
    fk << (*last_imu_msg_).linear_acceleration.x,
        (*last_imu_msg_).linear_acceleration.y,
        (*last_imu_msg_).linear_acceleration.z;

    wk << (*last_imu_msg_).angular_velocity.x,
        (*last_imu_msg_).angular_velocity.y,
        (*last_imu_msg_).angular_velocity.z;

    qk.w() = (*last_imu_msg_).orientation.w;
    qk.x() = (*last_imu_msg_).orientation.x;
    qk.y() = (*last_imu_msg_).orientation.y;
    qk.z() = (*last_imu_msg_).orientation.z;
    qk.normalize();
  }
}

void StateEstimator::readJointEncoder(
    const sensor_msgs::JointState::ConstPtr& last_joint_state_msg_,
    Eigen::VectorXd& jk) {
  if (last_joint_state_msg_ != NULL) {
    for (int i = 0; i < 12; i++) {
      jk[i] = (*last_joint_state_msg_).position[i];
    }
  }
}