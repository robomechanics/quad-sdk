#include "quad_controllers/quad_controller_base.hpp"

#include <algorithm>

namespace quad_controllers {

namespace {
const char* kNameTable[4][3] = {
    {"8", "0", "1"}, {"9", "2", "3"}, {"10", "4", "5"}, {"11", "6", "7"}};
}  // namespace

const char* QuadControllerBase::jointName(int leg, int j) {
  return kNameTable[leg][j];
}

controller_interface::CallbackReturn QuadControllerBase::on_init() {
  try {
    auto_declare<std::string>("interface_mode", "effort");
    auto_declare<std::string>("robot_namespace", "");
    auto_declare<std::vector<std::string>>(
        "joints", {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"});
    // Per joint-type (abad,hip,knee) torque clamp for effort mode. Reads the
    // existing motor_limits.torque (the same value QuadController clamped to) so
    // inverse-dynamics + PD torque can't blow past the motor limits and launch
    // the robot. Falls back to the legacy torque_limits name if present.
    auto_declare<std::vector<double>>("motor_limits.torque", {});
    auto_declare<std::vector<double>>("torque_limits", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return onInitExtra();
}

controller_interface::InterfaceConfiguration
QuadControllerBase::command_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto& j : joint_names_) {
    if (interface_mode_ == "motor") {
      names.push_back(j + "/position");
      names.push_back(j + "/velocity");
      names.push_back(j + "/kp");
      names.push_back(j + "/kd");
      names.push_back(j + "/effort");
    } else {
      names.push_back(j + "/effort");
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
QuadControllerBase::state_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto& j : joint_names_) {
    names.push_back(j + "/position");
    names.push_back(j + "/velocity");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn QuadControllerBase::on_configure(
    const rclcpp_lifecycle::State&) {
  auto& p = *get_node();
  interface_mode_ = p.get_parameter("interface_mode").as_string();
  robot_ns_ = p.get_parameter("robot_namespace").as_string();
  joint_names_ = p.get_parameter("joints").as_string_array();
  torque_limits_ = p.get_parameter("motor_limits.torque").as_double_array();
  if (torque_limits_.empty()) {  // legacy fallback
    torque_limits_ = p.get_parameter("torque_limits").as_double_array();
  }

  leg_command_pub_ =
      get_node()->create_publisher<quad_msgs::msg::LegCommandArray>(
          "control/joint_command", 1);

  return onConfigureExtra();
}

controller_interface::CallbackReturn QuadControllerBase::on_activate(
    const rclcpp_lifecycle::State&) {
  cmd_index_.clear();
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
    cmd_index_[command_interfaces_[i].get_prefix_name() + "/" +
               command_interfaces_[i].get_interface_name()] = i;
  state_index_.clear();
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
    state_index_[state_interfaces_[i].get_prefix_name() + "/" +
                 state_interfaces_[i].get_interface_name()] = i;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadControllerBase::on_deactivate(
    const rclcpp_lifecycle::State&) {
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadControllerBase::update(
    const rclcpp::Time&, const rclcpp::Duration&) {
  quad_msgs::msg::LegCommandArray out;
  out.leg_commands.resize(num_feet_);
  computeCommand(out);
  writeToInterfaces(out);
  if (leg_command_pub_) leg_command_pub_->publish(out);
  return controller_interface::return_type::OK;
}

double QuadControllerBase::measuredPosition(const std::string& joint) const {
  auto it = state_index_.find(joint + "/position");
  if (it == state_index_.end()) return 0.0;
  auto v = state_interfaces_[it->second].get_optional();
  return v.has_value() ? v.value() : 0.0;
}

double QuadControllerBase::measuredVelocity(const std::string& joint) const {
  auto it = state_index_.find(joint + "/velocity");
  if (it == state_index_.end()) return 0.0;
  auto v = state_interfaces_[it->second].get_optional();
  return v.has_value() ? v.value() : 0.0;
}

void QuadControllerBase::writeToInterfaces(
    const quad_msgs::msg::LegCommandArray& out) {
  auto set_cmd = [&](const std::string& joint, const std::string& iface,
                     double value) {
    auto it = cmd_index_.find(joint + "/" + iface);
    if (it != cmd_index_.end())
      (void)command_interfaces_[it->second].set_value(value);
  };
  const bool motor = (interface_mode_ == "motor");
  for (size_t l = 0; l < out.leg_commands.size() && l < 4; ++l) {
    for (size_t j = 0; j < out.leg_commands[l].motor_commands.size() && j < 3;
         ++j) {
      const auto& mc = out.leg_commands[l].motor_commands[j];
      const std::string jn = kNameTable[l][j];
      if (motor) {
        set_cmd(jn, "position", mc.pos_setpoint);
        set_cmd(jn, "velocity", mc.vel_setpoint);
        set_cmd(jn, "kp", mc.kp);
        set_cmd(jn, "kd", mc.kd);
        set_cmd(jn, "effort", mc.torque_ff);
      } else {
        double q = measuredPosition(jn), dq = measuredVelocity(jn);
        double tau = mc.torque_ff + mc.kp * (mc.pos_setpoint - q) +
                     mc.kd * (mc.vel_setpoint - dq);
        if (!torque_limits_.empty() &&
            static_cast<size_t>(j) < torque_limits_.size()) {
          const double lim = torque_limits_[j];
          tau = std::min(std::max(tau, -lim), lim);
        }
        set_cmd(jn, "effort", tau);
      }
    }
  }
}

}  // namespace quad_controllers
