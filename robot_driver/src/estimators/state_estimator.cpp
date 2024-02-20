#include "robot_driver/estimators/state_estimator.h"

StateEstimator::StateEstimator() {
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

void StateEstimator::readIMU(const sensor_msgs::Imu& last_imu_msg_,
                             Eigen::VectorXd& fk, Eigen::VectorXd& wk,
                             Eigen::Quaterniond& qk) {
  if (!last_imu_msg_.header.stamp.isZero()) {
    fk << (last_imu_msg_).linear_acceleration.x,
        (last_imu_msg_).linear_acceleration.y,
        (last_imu_msg_).linear_acceleration.z;

    wk << (last_imu_msg_).angular_velocity.x,
        (last_imu_msg_).angular_velocity.y, (last_imu_msg_).angular_velocity.z;

    qk.w() = (last_imu_msg_).orientation.w;
    qk.x() = (last_imu_msg_).orientation.x;
    qk.y() = (last_imu_msg_).orientation.y;
    qk.z() = (last_imu_msg_).orientation.z;
    qk.normalize();
  }
}

void StateEstimator::readJointEncoder(
    const sensor_msgs::JointState& last_joint_state_msg_, Eigen::VectorXd& jk,
    Eigen::VectorXd& vk) {
  if (!last_joint_state_msg_.header.stamp.isZero()) {
    for (int i = 0; i < 12; i++) {
      jk[i] = (last_joint_state_msg_).position[i];
      vk[i] = (last_joint_state_msg_).velocity[i];
    }
  }
}

void StateEstimator::loadMocapMsg(
    geometry_msgs::PoseStamped::ConstPtr last_mocap_msg) {
  last_mocap_msg_ = last_mocap_msg;
}

void StateEstimator::loadSensorMsg(
    sensor_msgs::Imu last_imu_msg,
    sensor_msgs::JointState last_joint_state_msg) {
  last_imu_msg_ = last_imu_msg;
  last_joint_state_msg_ = last_joint_state_msg;
}
