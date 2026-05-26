#include "robot_driver/estimators/state_estimator.hpp"

StateEstimator::StateEstimator(rclcpp::Node::SharedPtr node,
                               const std::string& robot_ns,
                               std::shared_ptr<quad_utils::QuadKD2> quadKD) {
  node_ = node;
  robot_ns_ = robot_ns;
  quadKD_ = quadKD;
}

void StateEstimator::init() {}

void StateEstimator::readIMU(
    const sensor_msgs::msg::Imu::SharedPtr& last_imu_msg_, Eigen::Vector3d& fk,
    Eigen::Vector3d& wk, Eigen::Quaterniond& qk) {
  if (last_imu_msg_) {
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
    const sensor_msgs::msg::JointState::SharedPtr& last_joint_state_msg,
    Eigen::VectorXd& jk) {
  if (last_joint_state_msg) {
    for (int i = 0; i < 12; i++) {
      jk[i] = (*last_joint_state_msg).position[i];
    }
  }
}

void StateEstimator::loadMocapMsg(
    geometry_msgs::msg::PoseStamped::SharedPtr last_mocap_msg) {
  last_mocap_msg_ = last_mocap_msg;
}

void StateEstimator::loadSensorMsg(
    sensor_msgs::msg::Imu last_imu_msg,
    sensor_msgs::msg::JointState last_joint_state_msg) {
  last_imu_msg_ = last_imu_msg;
  last_joint_state_msg_ = last_joint_state_msg;
}

void StateEstimator::loadFootContactMsg(
    const quad_msgs::msg::FootContact& msg) {
  last_foot_contact_msg_ = msg;
  foot_contact_received_ = true;
}
