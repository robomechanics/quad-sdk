#include "quad_hardware/unitree_system.hpp"

#include <unistd.h>

#include <cmath>
#include <limits>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quad_hardware {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// Sentinel q/dq that tell the Unitree motor driver to ignore position/velocity
// targets (matches UnitreeInterface::initLowCmd()).
constexpr float kIgnoreQ = 2.146e9f;
constexpr float kIgnoreDq = 16000.0f;
}  // namespace

int UnitreeSystem::motorIndexForJoint(const std::string& joint_name) {
  // quad-sdk numeric joint name -> Unitree LowCmd motor index.
  // Derived from UnitreeInterface::kLegMap + joint_names_ (leg-then-joint).
  static const std::unordered_map<std::string, int> kMap = {
      {"8", 3},  {"0", 4},  {"1", 5},   // FL: abad, hip, knee
      {"9", 9},  {"2", 10}, {"3", 11},  // RL
      {"10", 0}, {"4", 1},  {"5", 2},   // FR
      {"11", 6}, {"6", 7},  {"7", 8}};  // RR
  auto it = kMap.find(joint_name);
  return it == kMap.end() ? -1 : it->second;
}

hardware_interface::CallbackReturn UnitreeSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional hardware parameters from the URDF <hardware> block.
  if (auto it = info_.hardware_parameters.find("network_interface");
      it != info_.hardware_parameters.end()) {
    network_interface_ = it->second;
  }
  if (auto it = info_.hardware_parameters.find("read_only");
      it != info_.hardware_parameters.end()) {
    read_only_ = (it->second == "true" || it->second == "True" ||
                  it->second == "1");
  }

  const size_t n = info_.joints.size();
  if (n != static_cast<size_t>(kNumJoints)) {
    RCLCPP_ERROR(get_logger(),
                 "UnitreeSystem expects %d joints, URDF declares %zu",
                 kNumJoints, n);
    return hardware_interface::CallbackReturn::ERROR;
  }

  pos_state_.assign(n, kNaN);
  vel_state_.assign(n, kNaN);
  eff_state_.assign(n, kNaN);
  pos_cmd_.assign(n, kNaN);
  vel_cmd_.assign(n, kNaN);
  kp_cmd_.assign(n, kNaN);
  kd_cmd_.assign(n, kNaN);
  eff_cmd_.assign(n, kNaN);
  joint_motor_index_.assign(n, -1);

  for (size_t i = 0; i < n; ++i) {
    const auto& joint = info_.joints[i];
    int motor_idx = motorIndexForJoint(joint.name);
    if (motor_idx < 0) {
      RCLCPP_ERROR(get_logger(),
                   "UnitreeSystem: joint '%s' is not a known Go2 leg joint",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_motor_index_[i] = motor_idx;
  }

  // Resolve the IMU sensor name (first sensor declared), if present.
  if (!info_.sensors.empty()) {
    imu_sensor_name_ = info_.sensors.front().name;
  }

  RCLCPP_INFO(get_logger(),
              "UnitreeSystem initialized: %zu joints, iface=%s, read_only=%s",
              n, network_interface_.c_str(), read_only_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeSystem::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    // Disable sport mode so low-level commands are not overridden.
    unitree::robot::b2::MotionSwitcherClient msc;
    msc.SetTimeout(10.0f);
    msc.Init();
    auto motionActive = [&msc]() -> bool {
      std::string robotForm, motionName;
      int32_t ret = msc.CheckMode(robotForm, motionName);
      if (ret != 0) return true;
      return !motionName.empty();
    };
    int tries = 0;
    while (motionActive() && tries++ < 5) {
      RCLCPP_INFO(get_logger(), "Releasing Unitree motion control service...");
      msc.ReleaseMode();
      sleep(2);
    }

    cmd_pub_.reset(new unitree::robot::ChannelPublisher<
                   unitree_go::msg::dds_::LowCmd_>("rt/lowcmd"));
    cmd_pub_->InitChannel();

    state_sub_.reset(new unitree::robot::ChannelSubscriber<
                     unitree_go::msg::dds_::LowState_>("rt/lowstate"));
    state_sub_->InitChannel(
        std::bind(&UnitreeSystem::lowStateHandler, this, std::placeholders::_1),
        1);

    initLowCmd();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "UnitreeSystem DDS init failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "UnitreeSystem configured (DDS on %s)",
              network_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UnitreeSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION, &pos_state_[i]);
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_EFFORT, &eff_state_[i]);
  }

  // IMU sensor (consumed by imu_sensor_broadcaster).
  interfaces.emplace_back(imu_sensor_name_, "orientation.x",
                          &imu_orientation_[0]);
  interfaces.emplace_back(imu_sensor_name_, "orientation.y",
                          &imu_orientation_[1]);
  interfaces.emplace_back(imu_sensor_name_, "orientation.z",
                          &imu_orientation_[2]);
  interfaces.emplace_back(imu_sensor_name_, "orientation.w",
                          &imu_orientation_[3]);
  interfaces.emplace_back(imu_sensor_name_, "angular_velocity.x",
                          &imu_angular_velocity_[0]);
  interfaces.emplace_back(imu_sensor_name_, "angular_velocity.y",
                          &imu_angular_velocity_[1]);
  interfaces.emplace_back(imu_sensor_name_, "angular_velocity.z",
                          &imu_angular_velocity_[2]);
  interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.x",
                          &imu_linear_acceleration_[0]);
  interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.y",
                          &imu_linear_acceleration_[1]);
  interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.z",
                          &imu_linear_acceleration_[2]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // Five command interfaces per joint to drive the onboard motor PD:
    //   tau_motor = kp*(position - q_meas) + kd*(velocity - dq_meas) + effort
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
    interfaces.emplace_back(info_.joints[i].name, "kp", &kp_cmd_[i]);
    interfaces.emplace_back(info_.joints[i].name, "kd", &kd_cmd_[i]);
    interfaces.emplace_back(info_.joints[i].name,
                            hardware_interface::HW_IF_EFFORT, &eff_cmd_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn UnitreeSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Leave commands as NaN so write() stays in safe (zero-gain) mode until a
  // controller actually writes values.
  std::fill(pos_cmd_.begin(), pos_cmd_.end(), kNaN);
  std::fill(vel_cmd_.begin(), vel_cmd_.end(), kNaN);
  std::fill(kp_cmd_.begin(), kp_cmd_.end(), kNaN);
  std::fill(kd_cmd_.begin(), kd_cmd_.end(), kNaN);
  std::fill(eff_cmd_.begin(), eff_cmd_.end(), kNaN);
  RCLCPP_INFO(get_logger(), "UnitreeSystem activated%s",
              read_only_ ? " (READ-ONLY: no torque will be commanded)" : "");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  writeSafeCommand();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UnitreeSystem::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  writeSafeCommand();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeSystem::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!state_received_) {
    // No state yet — not an error during bring-up; leave NaNs in place.
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const int m = joint_motor_index_[i];
    pos_state_[i] = low_state_.motor_state()[m].q();
    vel_state_[i] = low_state_.motor_state()[m].dq();
    eff_state_[i] = low_state_.motor_state()[m].tau_est();
  }

  imu_orientation_[0] = low_state_.imu_state().quaternion()[0];  // x
  imu_orientation_[1] = low_state_.imu_state().quaternion()[1];  // y
  imu_orientation_[2] = low_state_.imu_state().quaternion()[2];  // z
  imu_orientation_[3] = low_state_.imu_state().quaternion()[3];  // w
  imu_angular_velocity_[0] = low_state_.imu_state().gyroscope()[0];
  imu_angular_velocity_[1] = low_state_.imu_state().gyroscope()[1];
  imu_angular_velocity_[2] = low_state_.imu_state().gyroscope()[2];
  imu_linear_acceleration_[0] = low_state_.imu_state().accelerometer()[0];
  imu_linear_acceleration_[1] = low_state_.imu_state().accelerometer()[1];
  imu_linear_acceleration_[2] = low_state_.imu_state().accelerometer()[2];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UnitreeSystem::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (read_only_) {
    writeSafeCommand();
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const int m = joint_motor_index_[i];
    // If a controller has not populated this joint's commands, stay safe.
    if (std::isnan(pos_cmd_[i]) || std::isnan(vel_cmd_[i]) ||
        std::isnan(kp_cmd_[i]) || std::isnan(kd_cmd_[i]) ||
        std::isnan(eff_cmd_[i])) {
      low_cmd_.motor_cmd()[m].mode() = 0x01;
      low_cmd_.motor_cmd()[m].q() = kIgnoreQ;
      low_cmd_.motor_cmd()[m].dq() = kIgnoreDq;
      low_cmd_.motor_cmd()[m].kp() = 0.0f;
      low_cmd_.motor_cmd()[m].kd() = 0.0f;
      low_cmd_.motor_cmd()[m].tau() = 0.0f;
      continue;
    }
    low_cmd_.motor_cmd()[m].mode() = 0x01;
    low_cmd_.motor_cmd()[m].q() = static_cast<float>(pos_cmd_[i]);
    low_cmd_.motor_cmd()[m].dq() = static_cast<float>(vel_cmd_[i]);
    low_cmd_.motor_cmd()[m].kp() = static_cast<float>(kp_cmd_[i]);
    low_cmd_.motor_cmd()[m].kd() = static_cast<float>(kd_cmd_[i]);
    low_cmd_.motor_cmd()[m].tau() = static_cast<float>(eff_cmd_[i]);
  }

  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
                             (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  if (cmd_pub_) cmd_pub_->Write(low_cmd_);
  return hardware_interface::return_type::OK;
}

void UnitreeSystem::writeSafeCommand() {
  initLowCmd();  // zero gains, ignore-sentinel targets => no torque
  low_cmd_.crc() = crc32Core(reinterpret_cast<uint32_t*>(&low_cmd_),
                             (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  if (cmd_pub_) cmd_pub_->Write(low_cmd_);
}

void UnitreeSystem::initLowCmd() {
  low_cmd_.head()[0] = 0xFE;
  low_cmd_.head()[1] = 0xEF;
  low_cmd_.level_flag() = 0xFF;
  low_cmd_.gpio() = 0;
  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd()[i].mode() = 0x01;
    low_cmd_.motor_cmd()[i].q() = kIgnoreQ;
    low_cmd_.motor_cmd()[i].dq() = kIgnoreDq;
    low_cmd_.motor_cmd()[i].kp() = 0;
    low_cmd_.motor_cmd()[i].kd() = 0;
    low_cmd_.motor_cmd()[i].tau() = 0;
  }
}

void UnitreeSystem::lowStateHandler(const void* message) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(message);
  state_received_ = true;
}

uint32_t UnitreeSystem::crc32Core(uint32_t* ptr, uint32_t len) {
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

}  // namespace quad_hardware

PLUGINLIB_EXPORT_CLASS(quad_hardware::UnitreeSystem,
                       hardware_interface::SystemInterface)
