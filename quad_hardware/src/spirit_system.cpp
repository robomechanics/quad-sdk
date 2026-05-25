#include "quad_hardware/spirit_system.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <unordered_map>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quad_hardware {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Mainboard telemetry map type returned by MBLink::get() (matches the legacy
// SpiritInterface typedef).
using MBData_t = std::unordered_map<std::string, Eigen::VectorXf>;

// Mainboard command packet, byte-identical to robot_driver SpiritInterface.
struct LimbCmd_t {
  Eigen::Vector3f pos, vel, tau;
  short kp[3];
  short kd[3];
  bool restart_flag;
};
}  // namespace

int SpiritSystem::motorIndexForJoint(const std::string& joint_name) {
  try {
    return std::stoi(joint_name);  // MBData index == numeric joint name
  } catch (...) {
    return -1;
  }
}

bool SpiritSystem::legJointForName(const std::string& n, int& leg, int& j) {
  // (leg, joint-in-leg) layout used by LimbCmd packing.
  static const std::unordered_map<std::string, std::pair<int, int>> kMap = {
      {"8", {0, 0}}, {"0", {0, 1}}, {"1", {0, 2}},    // FL: abad, hip, knee
      {"9", {1, 0}}, {"2", {1, 1}}, {"3", {1, 2}},    // RL
      {"10", {2, 0}}, {"4", {2, 1}}, {"5", {2, 2}},   // FR
      {"11", {3, 0}}, {"6", {3, 1}}, {"7", {3, 2}}};  // RR
  auto it = kMap.find(n);
  if (it == kMap.end()) return false;
  leg = it->second.first;
  j = it->second.second;
  return true;
}

double SpiritSystem::ktForJoint(const std::string& joint_name) {
  // Knees use the higher torque constant (matches kt_vec_ in SpiritInterface).
  if (joint_name == "1" || joint_name == "3" || joint_name == "5" ||
      joint_name == "7") {
    return 1.092;
  }
  return 0.546;
}

hardware_interface::CallbackReturn SpiritSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto it = info_.hardware_parameters.find("read_only");
      it != info_.hardware_parameters.end()) {
    read_only_ = (it->second == "true" || it->second == "True" ||
                  it->second == "1");
  }

  const size_t n = info_.joints.size();
  if (n != static_cast<size_t>(kNumJoints)) {
    RCLCPP_ERROR(get_logger(),
                 "SpiritSystem expects %d joints, URDF declares %zu",
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
  joint_leg_.assign(n, -1);
  joint_in_leg_.assign(n, -1);
  joint_kt_.assign(n, 0.546);

  for (size_t i = 0; i < n; ++i) {
    const std::string& name = info_.joints[i].name;
    int leg, j;
    if (motorIndexForJoint(name) < 0 || !legJointForName(name, leg, j)) {
      RCLCPP_ERROR(get_logger(),
                   "SpiritSystem: joint '%s' is not a known Spirit leg joint",
                   name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_motor_index_[i] = motorIndexForJoint(name);
    joint_leg_[i] = leg;
    joint_in_leg_[i] = j;
    joint_kt_[i] = ktForJoint(name);
  }

  if (!info_.sensors.empty()) {
    imu_sensor_name_ = info_.sensors.front().name;
  }

  RCLCPP_INFO(get_logger(), "SpiritSystem initialized: %zu joints, read_only=%s",
              n, read_only_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpiritSystem::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    int argc = 1;
    char prog[] = "quad_hardware_spirit";
    char* argv[] = {prog, nullptr};
    mblink_.start(argc, argv);
    mblink_.rxstart();
    mblink_.setRetry("_UPST_ADDRESS", 255);
    mblink_.setRetry("UPST_LOOP_DELAY", 1);
    mblink_started_ = true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "SpiritSystem MBLink start failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "SpiritSystem configured (MBLink up)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SpiritSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &pos_state_[i]);
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &vel_state_[i]);
    ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                        &eff_state_[i]);
  }
  ifaces.emplace_back(imu_sensor_name_, "orientation.x", &imu_orientation_[0]);
  ifaces.emplace_back(imu_sensor_name_, "orientation.y", &imu_orientation_[1]);
  ifaces.emplace_back(imu_sensor_name_, "orientation.z", &imu_orientation_[2]);
  ifaces.emplace_back(imu_sensor_name_, "orientation.w", &imu_orientation_[3]);
  ifaces.emplace_back(imu_sensor_name_, "angular_velocity.x",
                      &imu_angular_velocity_[0]);
  ifaces.emplace_back(imu_sensor_name_, "angular_velocity.y",
                      &imu_angular_velocity_[1]);
  ifaces.emplace_back(imu_sensor_name_, "angular_velocity.z",
                      &imu_angular_velocity_[2]);
  ifaces.emplace_back(imu_sensor_name_, "linear_acceleration.x",
                      &imu_linear_acceleration_[0]);
  ifaces.emplace_back(imu_sensor_name_, "linear_acceleration.y",
                      &imu_linear_acceleration_[1]);
  ifaces.emplace_back(imu_sensor_name_, "linear_acceleration.z",
                      &imu_linear_acceleration_[2]);
  return ifaces;
}

std::vector<hardware_interface::CommandInterface>
SpiritSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
    ifaces.emplace_back(info_.joints[i].name, "kp", &kp_cmd_[i]);
    ifaces.emplace_back(info_.joints[i].name, "kd", &kd_cmd_[i]);
    ifaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                        &eff_cmd_[i]);
  }
  return ifaces;
}

hardware_interface::CallbackReturn SpiritSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::fill(pos_cmd_.begin(), pos_cmd_.end(), kNaN);
  std::fill(vel_cmd_.begin(), vel_cmd_.end(), kNaN);
  std::fill(kp_cmd_.begin(), kp_cmd_.end(), kNaN);
  std::fill(kd_cmd_.begin(), kd_cmd_.end(), kNaN);
  std::fill(eff_cmd_.begin(), eff_cmd_.end(), kNaN);
  RCLCPP_INFO(get_logger(), "SpiritSystem activated%s",
              read_only_ ? " (READ-ONLY: no torque will be commanded)" : "");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpiritSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  sendSafeCommand();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpiritSystem::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  sendSafeCommand();
  if (mblink_started_) {
    try {
      mblink_.rxstop();
    } catch (...) {
    }
    mblink_started_ = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SpiritSystem::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!mblink_started_) return hardware_interface::return_type::OK;

  MBData_t mbdata = mblink_.get();  // may block briefly
  if (mbdata.empty()) {
    return hardware_interface::return_type::OK;  // no data yet during bring-up
  }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const int m = joint_motor_index_[i];
    pos_state_[i] = mbdata["joint_position"][m];
    vel_state_[i] = mbdata["joint_velocity"][m];
    eff_state_[i] = joint_kt_[i] * mbdata["joint_current"][m];
  }

  // Orientation: euler (roll, pitch, yaw) -> quaternion.
  const double r = mbdata["imu_euler"][0];
  const double p = mbdata["imu_euler"][1];
  const double y = mbdata["imu_euler"][2];
  const double cr = std::cos(r * 0.5), sr = std::sin(r * 0.5);
  const double cp = std::cos(p * 0.5), sp = std::sin(p * 0.5);
  const double cy = std::cos(y * 0.5), sy = std::sin(y * 0.5);
  imu_orientation_[0] = sr * cp * cy - cr * sp * sy;  // x
  imu_orientation_[1] = cr * sp * cy + sr * cp * sy;  // y
  imu_orientation_[2] = cr * cp * sy - sr * sp * cy;  // z
  imu_orientation_[3] = cr * cp * cy + sr * sp * sy;  // w

  imu_angular_velocity_[0] = mbdata["imu_angular_velocity"][0];
  imu_angular_velocity_[1] = mbdata["imu_angular_velocity"][1];
  imu_angular_velocity_[2] = mbdata["imu_angular_velocity"][2];

  // Linear acceleration is reported negated (matches SpiritInterface).
  imu_linear_acceleration_[0] = -mbdata["imu_linear_acceleration"][0];
  imu_linear_acceleration_[1] = -mbdata["imu_linear_acceleration"][1];
  imu_linear_acceleration_[2] = -mbdata["imu_linear_acceleration"][2];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SpiritSystem::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!mblink_started_) return hardware_interface::return_type::OK;

  if (read_only_) {
    sendSafeCommand();
    return hardware_interface::return_type::OK;
  }

  // Stay safe unless every joint has valid commands.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (std::isnan(pos_cmd_[i]) || std::isnan(vel_cmd_[i]) ||
        std::isnan(kp_cmd_[i]) || std::isnan(kd_cmd_[i]) ||
        std::isnan(eff_cmd_[i])) {
      sendSafeCommand();
      return hardware_interface::return_type::OK;
    }
  }

  LimbCmd_t limbcmd[kNumLegs];
  std::memset(limbcmd, 0, sizeof(limbcmd));
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const int leg = joint_leg_[i];
    const int j = joint_in_leg_[i];
    limbcmd[leg].pos[j] = static_cast<float>(pos_cmd_[i]);
    limbcmd[leg].vel[j] = static_cast<float>(vel_cmd_[i]);
    limbcmd[leg].tau[j] = static_cast<float>(eff_cmd_[i]);
    limbcmd[leg].kp[j] = static_cast<short>(kp_cmd_[i]);
    limbcmd[leg].kd[j] = static_cast<short>(kd_cmd_[i]);
    limbcmd[leg].restart_flag = false;
  }

  float data[58] = {0};
  std::memcpy(data, limbcmd, kNumLegs * sizeof(LimbCmd_t));
  mblink_.sendUser(Eigen::Map<const Eigen::Matrix<float, 58, 1>>(data));
  return hardware_interface::return_type::OK;
}

void SpiritSystem::sendSafeCommand() {
  if (!mblink_started_) return;
  LimbCmd_t limbcmd[kNumLegs];
  std::memset(limbcmd, 0, sizeof(limbcmd));  // zero pos/vel/tau/kp/kd => no torque
  float data[58] = {0};
  std::memcpy(data, limbcmd, kNumLegs * sizeof(LimbCmd_t));
  mblink_.sendUser(Eigen::Map<const Eigen::Matrix<float, 58, 1>>(data));
}

}  // namespace quad_hardware

PLUGINLIB_EXPORT_CLASS(quad_hardware::SpiritSystem,
                       hardware_interface::SystemInterface)
