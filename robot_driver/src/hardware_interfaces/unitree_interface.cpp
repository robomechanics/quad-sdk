#include "robot_driver/hardware_interfaces/unitree_interface.hpp"

constexpr int
    UnitreeInterface::kLegMap[kNumLegs][kJointsPerLeg];

UnitreeInterface::UnitreeInterface() : HardwareInterface() {}

void UnitreeInterface::loadInterface(int argc, char** argv) {
  // argv[0] is the node name; the network interface (e.g. "eth0")
  // should be passed as a command-line argument or default to "eth0".
  std::string net_iface = "eth0";

  unitree::robot::ChannelFactory::Instance()->Init(0,
                                                   net_iface);

  // Create publisher on rt/lowcmd
  cmd_pub_.reset(
      new unitree::robot::ChannelPublisher<
          unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
  cmd_pub_->InitChannel();

  // Create subscriber on rt/lowstate
  state_sub_.reset(
      new unitree::robot::ChannelSubscriber<
          unitree_go::msg::dds_::LowState_>("rt/lowstate"));
  state_sub_->InitChannel(
      std::bind(&UnitreeInterface::lowStateHandler, this,
                std::placeholders::_1),
      1);

  initLowCmd();
}

void UnitreeInterface::unloadInterface() {
  // Zero out commands before shutting down
  initLowCmd();
  low_cmd_.crc() = crc32Core(
      reinterpret_cast<uint32_t*>(&low_cmd_),
      (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  cmd_pub_->Write(low_cmd_);
}

void UnitreeInterface::initLowCmd() {
  low_cmd_.head()[0] = 0xFE;
  low_cmd_.head()[1] = 0xEF;
  low_cmd_.level_flag() = 0xFF;
  low_cmd_.gpio() = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd()[i].mode() = 0x01;
    low_cmd_.motor_cmd()[i].q() = 2.146E+9f;
    low_cmd_.motor_cmd()[i].dq() = 16000.0f;
    low_cmd_.motor_cmd()[i].kp() = 0;
    low_cmd_.motor_cmd()[i].kd() = 0;
    low_cmd_.motor_cmd()[i].tau() = 0;
  }
}

void UnitreeInterface::lowStateHandler(const void* message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  low_state_ =
      *(const unitree_go::msg::dds_::LowState_*)message;
  state_received_ = true;
}

uint32_t UnitreeInterface::crc32Core(uint32_t* ptr,
                                     uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }
      if (data & xbit) {
        CRC32 ^= dwPolynomial;
      }
      xbit >>= 1;
    }
  }
  return CRC32;
}

bool UnitreeInterface::send(
    const quad_msgs::msg::LegCommandArray&
        leg_command_array_msg,
    const Eigen::VectorXd& /*user_tx_data*/) {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto& leg_cmd =
        leg_command_array_msg.leg_commands.at(leg);

    for (int j = 0; j < kJointsPerLeg; ++j) {
      int motor_idx = kLegMap[leg][j];
      const auto& mc = leg_cmd.motor_commands.at(j);

      low_cmd_.motor_cmd()[motor_idx].mode() = 0x01;
      low_cmd_.motor_cmd()[motor_idx].q() =
          static_cast<float>(mc.pos_setpoint);
      low_cmd_.motor_cmd()[motor_idx].dq() =
          static_cast<float>(mc.vel_setpoint);
      low_cmd_.motor_cmd()[motor_idx].tau() =
          static_cast<float>(mc.torque_ff);
      low_cmd_.motor_cmd()[motor_idx].kp() =
          static_cast<float>(mc.kp);
      low_cmd_.motor_cmd()[motor_idx].kd() =
          static_cast<float>(mc.kd);
    }
  }

  // CRC must be computed before every publish
  low_cmd_.crc() = crc32Core(
      reinterpret_cast<uint32_t*>(&low_cmd_),
      (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

  cmd_pub_->Write(low_cmd_);
  return true;
}

bool UnitreeInterface::recv(
    sensor_msgs::msg::JointState& joint_state_msg,
    sensor_msgs::msg::Imu& imu_msg,
    Eigen::VectorXd& /*user_rx_data*/) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!state_received_) {
    std::cout << "Not Receiving State Information from Unitree Robot" << std::endl;
    return false;
  }

  // Fill joint state: iterate legs in quad-sdk order
  int idx = 0;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    for (int j = 0; j < kJointsPerLeg; ++j) {
      int motor_idx = kLegMap[leg][j];
      joint_state_msg.name[idx] = joint_names_[idx];
      joint_state_msg.position[idx] =
          low_state_.motor_state()[motor_idx].q();
      joint_state_msg.velocity[idx] =
          low_state_.motor_state()[motor_idx].dq();
      joint_state_msg.effort[idx] =
          low_state_.motor_state()[motor_idx].tau_est();
      idx++;
    }
  }

  // IMU — Unitree provides quaternion directly
  imu_msg.orientation.x =
      low_state_.imu_state().quaternion()[0];
  imu_msg.orientation.y =
      low_state_.imu_state().quaternion()[1];
  imu_msg.orientation.z =
      low_state_.imu_state().quaternion()[2];
  imu_msg.orientation.w =
      low_state_.imu_state().quaternion()[3];

  imu_msg.angular_velocity.x =
      low_state_.imu_state().gyroscope()[0];
  imu_msg.angular_velocity.y =
      low_state_.imu_state().gyroscope()[1];
  imu_msg.angular_velocity.z =
      low_state_.imu_state().gyroscope()[2];

  imu_msg.linear_acceleration.x =
      low_state_.imu_state().accelerometer()[0];
  imu_msg.linear_acceleration.y =
      low_state_.imu_state().accelerometer()[1];
  imu_msg.linear_acceleration.z =
      low_state_.imu_state().accelerometer()[2];

  return true;
}
