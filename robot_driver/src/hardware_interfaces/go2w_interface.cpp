#include "robot_driver/hardware_interfaces/go2w_interface.hpp"

constexpr int Go2WInterface::kWheelMap[kNumWheels];

Go2WInterface::Go2WInterface() : Go2Interface() {}

bool Go2WInterface::send(
    const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data) {
  // Pack leg motor commands (indices 0-11) using base-class mapping.
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto& leg_cmd = leg_command_array_msg.leg_commands.at(leg);

    for (int j = 0; j < kJointsPerLeg; ++j) {
      int motor_idx = kLegMap[leg][j];
      const auto& mc = leg_cmd.motor_commands.at(j);

      low_cmd_.motor_cmd()[motor_idx].mode() = 0x01;
      low_cmd_.motor_cmd()[motor_idx].q() = static_cast<float>(mc.pos_setpoint);
      low_cmd_.motor_cmd()[motor_idx].dq() =
          static_cast<float>(mc.vel_setpoint);
      low_cmd_.motor_cmd()[motor_idx].tau() = static_cast<float>(mc.torque_ff);
      low_cmd_.motor_cmd()[motor_idx].kp() = static_cast<float>(mc.kp);
      low_cmd_.motor_cmd()[motor_idx].kd() = static_cast<float>(mc.kd);
    }
  }

  // Pack wheel velocity commands (indices 12-15).
  // Wheels are velocity-controlled: kp=0, q=0, kd>0, dq=target.
  const bool have_wheel_cmd = (user_tx_data.size() == kWheelCmdSize);
  for (int leg = 0; leg < kNumWheels; ++leg) {
    int motor_idx = kWheelMap[leg];

    float vel = 0.0f;
    float kd = kDefaultWheelKd;
    float tau_ff = 0.0f;
    if (have_wheel_cmd) {
      const int base = kTxRestartFlagOffset + leg * kWheelCmdFields;
      vel = static_cast<float>(user_tx_data[base + 0]);
      kd = static_cast<float>(user_tx_data[base + 1]);
      tau_ff = static_cast<float>(user_tx_data[base + 2]);
    }

    low_cmd_.motor_cmd()[motor_idx].mode() = 0x01;
    low_cmd_.motor_cmd()[motor_idx].q() = 0.0f;
    low_cmd_.motor_cmd()[motor_idx].kp() = 0.0f;
    low_cmd_.motor_cmd()[motor_idx].dq() = vel;
    low_cmd_.motor_cmd()[motor_idx].kd() = kd;
    low_cmd_.motor_cmd()[motor_idx].tau() = tau_ff;
  }

  // CRC must be computed before every publish.
  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
                             (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

  cmd_pub_->Write(low_cmd_);
  return true;
}

bool Go2WInterface::recv(sensor_msgs::msg::JointState& joint_state_msg,
                         sensor_msgs::msg::Imu& imu_msg,
                         Eigen::VectorXd& user_rx_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!state_received_) {
    std::cout << "Not receiving state information from Go2-W Interface"
              << std::endl;
    return false;
  }

  // Joint state for the 12 leg motors.
  int idx = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    for (int j = 0; j < kJointsPerLeg; ++j) {
      int motor_idx = kLegMap[leg][j];
      joint_state_msg.name[idx] = joint_names_[idx];
      joint_state_msg.position[idx] = low_state_.motor_state()[motor_idx].q();
      joint_state_msg.velocity[idx] = low_state_.motor_state()[motor_idx].dq();
      joint_state_msg.effort[idx] =
          low_state_.motor_state()[motor_idx].tau_est();
      idx++;
    }
  }

  // Joint state for the 4 wheel motors. Caller must size joint_state_msg
  // to kNumJoints + kNumWheels (16) when running on a Go2-W.
  if (static_cast<int>(joint_state_msg.name.size()) >=
      kNumJoints + kNumWheels) {
    for (int leg = 0; leg < kNumWheels; ++leg) {
      int motor_idx = kWheelMap[leg];
      joint_state_msg.name[idx] = wheel_joint_names_[leg];
      joint_state_msg.position[idx] = low_state_.motor_state()[motor_idx].q();
      joint_state_msg.velocity[idx] = low_state_.motor_state()[motor_idx].dq();
      joint_state_msg.effort[idx] =
          low_state_.motor_state()[motor_idx].tau_est();
      idx++;
    }
  }

  // Mirror wheel state into user_rx_data for downstream consumers that
  // bypass JointState.
  if (user_rx_data.size() != kWheelStateSize) {
    user_rx_data.resize(kWheelStateSize);
  }
  for (int leg = 0; leg < kNumWheels; ++leg) {
    int motor_idx = kWheelMap[leg];
    user_rx_data[leg * kWheelStateFields + 0] =
        low_state_.motor_state()[motor_idx].q();
    user_rx_data[leg * kWheelStateFields + 1] =
        low_state_.motor_state()[motor_idx].dq();
    user_rx_data[leg * kWheelStateFields + 2] =
        low_state_.motor_state()[motor_idx].tau_est();
  }

  // IMU — Unitree provides quaternion directly.
  imu_msg.orientation.x = low_state_.imu_state().quaternion()[0];
  imu_msg.orientation.y = low_state_.imu_state().quaternion()[1];
  imu_msg.orientation.z = low_state_.imu_state().quaternion()[2];
  imu_msg.orientation.w = low_state_.imu_state().quaternion()[3];

  imu_msg.angular_velocity.x = low_state_.imu_state().gyroscope()[0];
  imu_msg.angular_velocity.y = low_state_.imu_state().gyroscope()[1];
  imu_msg.angular_velocity.z = low_state_.imu_state().gyroscope()[2];

  imu_msg.linear_acceleration.x = low_state_.imu_state().accelerometer()[0];
  imu_msg.linear_acceleration.y = low_state_.imu_state().accelerometer()[1];
  imu_msg.linear_acceleration.z = low_state_.imu_state().accelerometer()[2];

  return true;
}
