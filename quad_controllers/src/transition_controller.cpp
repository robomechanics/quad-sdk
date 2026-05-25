#include "quad_controllers/transition_controller.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

namespace quad_controllers {

controller_interface::CallbackReturn TransitionController::onInitExtra() {
  try {
    auto_declare<std::vector<double>>("from_angles", {});
    auto_declare<std::vector<double>>("to_angles", {});
    auto_declare<double>("duration", 1.0);
    auto_declare<std::vector<double>>("kp", {});
    auto_declare<std::vector<double>>("kd", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "onInitExtra failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TransitionController::onConfigureExtra() {
  auto& p = *get_node();
  from_angles_ = p.get_parameter("from_angles").as_double_array();
  to_angles_ = p.get_parameter("to_angles").as_double_array();
  duration_ = p.get_parameter("duration").as_double();
  kp_ = p.get_parameter("kp").as_double_array();
  kd_ = p.get_parameter("kd").as_double_array();
  RCLCPP_INFO(get_node()->get_logger(),
              "TransitionController configured (duration=%.3f, mode=%s)",
              duration_, interface_mode_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TransitionController::on_activate(
    const rclcpp_lifecycle::State& state) {
  auto ret = QuadControllerBase::on_activate(state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) return ret;
  start_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool TransitionController::computeCommand(
    quad_msgs::msg::LegCommandArray& out) {
  double t = 0.0;
  if (duration_ > 0.0)
    t = (get_node()->now() - start_time_).seconds() / duration_;
  t = std::min(std::max(t, 0.0), 1.0);

  for (int i = 0; i < num_feet_; ++i) {
    out.leg_commands.at(i).motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {
      auto& mc = out.leg_commands.at(i).motor_commands.at(j);
      mc.pos_setpoint =
          from_angles_.at(j) + (to_angles_.at(j) - from_angles_.at(j)) * t;
      mc.vel_setpoint = 0;
      mc.torque_ff = 0;
      mc.kp = kp_.at(j);
      mc.kd = kd_.at(j);
    }
  }
  return true;
}

}  // namespace quad_controllers

PLUGINLIB_EXPORT_CLASS(quad_controllers::TransitionController,
                       controller_interface::ControllerInterface)
