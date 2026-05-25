#include "quad_controllers/safety_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace quad_controllers {

controller_interface::CallbackReturn SafetyController::onInitExtra() {
  try {
    auto_declare<std::vector<double>>("kp", {});
    auto_declare<std::vector<double>>("kd", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "onInitExtra failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyController::onConfigureExtra() {
  auto& p = *get_node();
  kp_ = p.get_parameter("kp").as_double_array();
  kd_ = p.get_parameter("kd").as_double_array();
  RCLCPP_INFO(get_node()->get_logger(), "SafetyController configured (mode=%s)",
              interface_mode_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SafetyController::computeCommand(quad_msgs::msg::LegCommandArray& out) {
  for (int i = 0; i < num_feet_; ++i) {
    out.leg_commands.at(i).motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {
      auto& mc = out.leg_commands.at(i).motor_commands.at(j);
      mc.pos_setpoint = 0;
      mc.vel_setpoint = 0;
      mc.torque_ff = 0;
      mc.kp = kp_.at(j);
      mc.kd = kd_.at(j);
    }
  }
  return true;
}

}  // namespace quad_controllers

PLUGINLIB_EXPORT_CLASS(quad_controllers::SafetyController,
                       controller_interface::ControllerInterface)
