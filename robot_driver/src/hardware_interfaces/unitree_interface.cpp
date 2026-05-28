#include "robot_driver/hardware_interfaces/unitree_interface.hpp"

#include <cstdlib>
#include <iostream>
#include <unistd.h>

constexpr int UnitreeInterface::kLegMap[UnitreeInterface::kNumLegs]
                                       [UnitreeInterface::kJointsPerLeg];
constexpr int UnitreeInterface::kWheelMap[UnitreeInterface::kNumWheels];

UnitreeInterface::UnitreeInterface(const std::string& robot_name)
    : HardwareInterface(),
      robot_name_(robot_name),
      num_motors_(kNumJoints),
      has_wheels_(false) {
  if (robot_name_ == "go2w") {
    num_motors_ = kNumJoints + kNumWheels;
    has_wheels_ = true;
  } else if (robot_name_ != "go2") {
    std::cerr << "UnitreeInterface: unknown robot_name='" << robot_name_
              << "', falling back to Go2." << std::endl;
    robot_name_ = "go2";
  }
}

void UnitreeInterface::loadInterface(int /*argc*/, char** /*argv*/) {
  // ROBOT_MCU_IFACE is the interface NAME (e.g. "enP8p1s0") on the MCU subnet
  // (192.168.123.0/24); init_robot.sh detects and exports it. The Unitree SDK
  // wants a name here, not an IP. Falls back to eth0 if unset.
  const char* env_iface = std::getenv("ROBOT_MCU_IFACE");
  std::string net_iface = (env_iface && *env_iface) ? env_iface : "eth0";
  std::cout << "UnitreeInterface: using network interface '" << net_iface
            << "' for MCU DDS domain." << std::endl;

  unitree::robot::ChannelFactory::Instance()->Init(0, net_iface);

  // Disable sport mode so low-level commands are not overridden.
  // Mirrors the retry loop in Unitree's go2_stand_example.
  unitree::robot::b2::MotionSwitcherClient msc;
  msc.SetTimeout(10.0f);
  msc.Init();

  auto queryMotionStatus = [&msc]() -> bool {
    std::string robotForm, motionName;
    int32_t ret = msc.CheckMode(robotForm, motionName);
    if (ret != 0) {
      std::cout << "CheckMode failed. Error code: " << ret << std::endl;
      return true;
    }
    if (motionName.empty()) {
      std::cout << "Motion control service deactivated." << std::endl;
      return false;
    }
    std::cout << "Active motion service: " << motionName << std::endl;
    return true;
  };

  while (queryMotionStatus()) {
    std::cout << "Releasing motion control service..." << std::endl;
    int32_t ret = msc.ReleaseMode();
    if (ret == 0) {
      std::cout << "ReleaseMode succeeded." << std::endl;
    } else {
      std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    }
    sleep(5);
  }

  cmd_pub_.reset(new unitree::robot::ChannelPublisher<
                 unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
  cmd_pub_->InitChannel();

  state_sub_.reset(new unitree::robot::ChannelSubscriber<
                   unitree_go::msg::dds_::LowState_>("rt/lowstate"));
  state_sub_->InitChannel(std::bind(&UnitreeInterface::lowStateHandler, this,
                                    std::placeholders::_1),
                          1);

  initLowCmd();
}

void UnitreeInterface::unloadInterface() {
  initLowCmd();
  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
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
  low_state_ = *(const unitree_go::msg::dds_::LowState_*)message;
  // Reorder Unitree's foot_force [FR, FL, RR, RL] into quad-sdk leg order
  // [FL, RL, FR, RR] so downstream code uses a single convention.
  const auto& ff = low_state_.foot_force();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    foot_force_quad_order_[leg] = ff[kFootForceMap[leg]];
  }
  state_received_ = true;
}

std::array<int16_t, UnitreeInterface::kNumLegs>
UnitreeInterface::getFootForcesRaw() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return foot_force_quad_order_;
}

uint32_t UnitreeInterface::crc32Core(uint32_t* ptr, uint32_t len) {
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
    const quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data) {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto& leg_cmd = leg_command_array_msg.leg_commands.at(leg);

    // 12 leg motors.
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

    // 4 wheel motors (Go2-W only).
    if (has_wheels_) {
      int wheel_idx = kWheelMap[leg];

      const bool have_per_leg_wheel_cmd =
          (static_cast<int>(leg_cmd.motor_commands.size()) > kJointsPerLeg);
      const bool have_legacy_tx =
          (user_tx_data.size() == kWheelTxSize);

      float q = 0.0f;
      float vel = 0.0f;
      float kp = 0.0f;
      float kd = kDefaultWheelKd;
      float tau_ff = 0.0f;

      if (have_per_leg_wheel_cmd) {
        const auto& mc = leg_cmd.motor_commands.at(kJointsPerLeg);
        q = static_cast<float>(mc.pos_setpoint);
        vel = static_cast<float>(mc.vel_setpoint);
        kp = static_cast<float>(mc.kp);
        kd = static_cast<float>(mc.kd);
        tau_ff = static_cast<float>(mc.torque_ff);
      } else if (have_legacy_tx) {
        const int base = kTxRestartFlagOffset + leg * kWheelCmdFields;
        vel = static_cast<float>(user_tx_data[base + 0]);
        kd = static_cast<float>(user_tx_data[base + 1]);
        tau_ff = static_cast<float>(user_tx_data[base + 2]);
      }

      low_cmd_.motor_cmd()[wheel_idx].mode() = 0x01;
      low_cmd_.motor_cmd()[wheel_idx].q() = q;
      low_cmd_.motor_cmd()[wheel_idx].dq() = vel;
      low_cmd_.motor_cmd()[wheel_idx].kp() = kp;
      low_cmd_.motor_cmd()[wheel_idx].kd() = kd;
      low_cmd_.motor_cmd()[wheel_idx].tau() = tau_ff;
    }
  }

  // CRC must be computed before every publish.
  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
                             (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  cmd_pub_->Write(low_cmd_);
  return true;
}

bool UnitreeInterface::recv(sensor_msgs::msg::JointState& joint_state_msg,
                            sensor_msgs::msg::Imu& imu_msg,
                            Eigen::VectorXd& user_rx_data) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!state_received_) {
    std::cout << "Not receiving state from UnitreeInterface" << std::endl;
    return false;
  }

  // 12 leg joints, in quad-sdk leg-then-joint order.
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

  // 4 wheel joints, but only if caller has sized JointState for it.
  if (has_wheels_ &&
      static_cast<int>(joint_state_msg.name.size()) >=
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
  // bypass JointState (preserves Go2WInterface behavior).
  if (has_wheels_) {
    if (user_rx_data.size() != kWheelRxSize) {
      user_rx_data.resize(kWheelRxSize);
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
  }

  imu_msg.orientation.w = low_state_.imu_state().quaternion()[0];
  imu_msg.orientation.x = low_state_.imu_state().quaternion()[1];
  imu_msg.orientation.y = low_state_.imu_state().quaternion()[2];
  imu_msg.orientation.z = low_state_.imu_state().quaternion()[3];

  imu_msg.angular_velocity.x = low_state_.imu_state().gyroscope()[0];
  imu_msg.angular_velocity.y = low_state_.imu_state().gyroscope()[1];
  imu_msg.angular_velocity.z = low_state_.imu_state().gyroscope()[2];

  imu_msg.linear_acceleration.x = low_state_.imu_state().accelerometer()[0];
  imu_msg.linear_acceleration.y = low_state_.imu_state().accelerometer()[1];
  imu_msg.linear_acceleration.z = low_state_.imu_state().accelerometer()[2];

  return true;
}
