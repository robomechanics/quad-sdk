#include "gazebo_plugins/quad_wheel_controller_plugin.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <array>
#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {

namespace {

// Mirrors load_joint_names_from_robot_yaml() in controller_plugin.cpp:
// produces 12 leg joint names in the same order
// (hip0, knee0, hip1, knee1, hip2, knee2, hip3, knee3, abad0..abad3).
bool load_leg_joint_names(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
    std::vector<std::string>& joint_names) {
  std::array<std::vector<std::string>, 4> leg_joint_names;
  for (size_t leg_idx = 0; leg_idx < leg_joint_names.size(); ++leg_idx) {
    const std::string leg_ns = "leg_" + std::to_string(leg_idx);
    std::string abad_name;
    std::string hip_name;
    std::string knee_name;

    if (node->get_parameter(leg_ns + ".joints.abad.name", abad_name) &&
        node->get_parameter(leg_ns + ".joints.hip.name", hip_name) &&
        node->get_parameter(leg_ns + ".joints.knee.name", knee_name)) {
      if (!abad_name.empty() && !hip_name.empty() && !knee_name.empty()) {
        leg_joint_names[leg_idx] = {abad_name, hip_name, knee_name};
        continue;
      }
    }

    const std::string param_name = leg_ns + ".joint_names";
    if (!node->get_parameter(param_name, leg_joint_names[leg_idx])) {
      return false;
    }

    if (leg_joint_names[leg_idx].size() != 3) {
      RCLCPP_ERROR(node->get_logger(),
                   "Parameter '%s' must contain exactly 3 joint names",
                   param_name.c_str());
      return false;
    }
  }

  joint_names = {
      leg_joint_names[0][1], leg_joint_names[0][2], leg_joint_names[1][1],
      leg_joint_names[1][2], leg_joint_names[2][1], leg_joint_names[2][2],
      leg_joint_names[3][1], leg_joint_names[3][2], leg_joint_names[0][0],
      leg_joint_names[1][0], leg_joint_names[2][0], leg_joint_names[3][0]};
  return true;
}

bool validate_motor_limits_3(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
    const std::vector<double>& torque_lims,
    const std::vector<double>& speed_lims) {
  constexpr size_t kExpectedMotorDims = 3;
  if (torque_lims.size() != kExpectedMotorDims) {
    RCLCPP_ERROR(node->get_logger(),
                 "Parameter 'motor_limits.torque' must have %zu values",
                 kExpectedMotorDims);
    return false;
  }
  if (speed_lims.size() != kExpectedMotorDims) {
    RCLCPP_ERROR(node->get_logger(),
                 "Parameter 'motor_limits.speed' must have %zu values",
                 kExpectedMotorDims);
    return false;
  }
  return true;
}

bool load_leg_motor_limits(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
    std::vector<double>& torque_lims, std::vector<double>& speed_lims) {
  bool found_torque = node->get_parameter("motor_limits.torque", torque_lims);
  bool found_speed = node->get_parameter("motor_limits.speed", speed_lims);
  if (!found_torque) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to get parameter 'motor_limits.torque'");
    return false;
  }
  if (!found_speed) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to get parameter 'motor_limits.speed'");
    return false;
  }
  return true;
}

}  // namespace

QuadWheelController::QuadWheelController() {}

QuadWheelController::~QuadWheelController() { sub_command_.reset(); }

bool QuadWheelController::load_wheel_joint_names(
    std::vector<std::string>& wheel_names) const {
  wheel_names.clear();
  wheel_names.reserve(kNumWheels);
  for (int leg_idx = 0; leg_idx < kNumWheels; ++leg_idx) {
    const std::string leg_ns = "leg_" + std::to_string(leg_idx);
    std::string wheel_name;
    if (!node_->get_parameter(leg_ns + ".joints.wheel.name", wheel_name) ||
        wheel_name.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Missing required parameter '%s.joints.wheel.name' for "
                   "QuadWheelController",
                   leg_ns.c_str());
      return false;
    }
    wheel_names.push_back(wheel_name);
  }
  return true;
}

bool QuadWheelController::load_wheel_motor_limits(
    std::vector<double>& torque_lims, std::vector<double>& speed_lims) const {
  // Wheel limits are optional; default to a generous fallback that mirrors
  // the per-leg knee limit so we never command more than the SDK would
  // accept on the real robot.
  bool found_torque =
      node_->get_parameter("wheel_motor_limits.torque", torque_lims);
  bool found_speed =
      node_->get_parameter("wheel_motor_limits.speed", speed_lims);
  if (!found_torque || torque_lims.empty()) {
    torque_lims = {25.0};
    RCLCPP_WARN(node_->get_logger(),
                "Parameter 'wheel_motor_limits.torque' missing; using default "
                "%.1f Nm",
                torque_lims[0]);
  }
  if (!found_speed || speed_lims.empty()) {
    speed_lims = {30.0};
    RCLCPP_WARN(node_->get_logger(),
                "Parameter 'wheel_motor_limits.speed' missing; using default "
                "%.1f rad/s",
                speed_lims[0]);
  }
  return true;
}

controller_interface::InterfaceConfiguration
QuadWheelController::command_interface_configuration() const {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names_) {
    interfaces.push_back(joint + "/effort");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interfaces};
}

controller_interface::InterfaceConfiguration
QuadWheelController::state_interface_configuration() const {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names_) {
    interfaces.push_back(joint + "/position");
    interfaces.push_back(joint + "/velocity");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interfaces};
}

controller_interface::CallbackReturn QuadWheelController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  sub_command_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadWheelController::on_init() {
  // Leg joints (indices 0..11) follow the same per-leg motor mapping as
  // QuadController.
  leg_map_[0] = std::make_pair(0, 1);    // hip0
  leg_map_[1] = std::make_pair(0, 2);    // knee0
  leg_map_[2] = std::make_pair(1, 1);    // hip1
  leg_map_[3] = std::make_pair(1, 2);    // knee1
  leg_map_[4] = std::make_pair(2, 1);    // hip2
  leg_map_[5] = std::make_pair(2, 2);    // knee2
  leg_map_[6] = std::make_pair(3, 1);    // hip3
  leg_map_[7] = std::make_pair(3, 2);    // knee3
  leg_map_[8] = std::make_pair(0, 0);    // abad0
  leg_map_[9] = std::make_pair(1, 0);    // abad1
  leg_map_[10] = std::make_pair(2, 0);   // abad2
  leg_map_[11] = std::make_pair(3, 0);   // abad3
  // Wheel joints (indices 12..15): motor index 3 within each LegCommand.
  leg_map_[12] = std::make_pair(0, 3);   // wheel0
  leg_map_[13] = std::make_pair(1, 3);   // wheel1
  leg_map_[14] = std::make_pair(2, 3);   // wheel2
  leg_map_[15] = std::make_pair(3, 3);   // wheel3

  node_ = get_node();
  node_->declare_parameter<std::vector<std::string>>(
      "joints", std::vector<std::string>{});
  for (int leg_idx = 0; leg_idx < kNumLegs; ++leg_idx) {
    const std::string leg_ns = "leg_" + std::to_string(leg_idx);
    node_->declare_parameter<std::vector<std::string>>(
        leg_ns + ".joint_names", std::vector<std::string>{});
    node_->declare_parameter<std::string>(leg_ns + ".joints.abad.name", "");
    node_->declare_parameter<std::string>(leg_ns + ".joints.hip.name", "");
    node_->declare_parameter<std::string>(leg_ns + ".joints.knee.name", "");
    node_->declare_parameter<std::string>(leg_ns + ".joints.wheel.name", "");
  }
  node_->declare_parameter<std::string>("topics.control.joint_command", "");
  node_->declare_parameter<std::vector<double>>("motor_limits.torque",
                                                std::vector<double>{});
  node_->declare_parameter<std::vector<double>>("motor_limits.speed",
                                                std::vector<double>{});
  node_->declare_parameter<std::vector<double>>("wheel_motor_limits.torque",
                                                std::vector<double>{});
  node_->declare_parameter<std::vector<double>>("wheel_motor_limits.speed",
                                                std::vector<double>{});

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadWheelController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  std::string urdf_string = get_robot_description();

  urdf::Model urdf;
  if (!urdf.initString(urdf_string)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse urdf file");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::vector<std::string> leg_names;
  if (!load_leg_joint_names(node_, leg_names)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to load leg joint names (namespace: %s)",
                 node_->get_namespace());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::vector<std::string> wheel_names;
  if (!load_wheel_joint_names(wheel_names)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  joint_names_.insert(joint_names_.end(), leg_names.begin(), leg_names.end());
  joint_names_.insert(joint_names_.end(), wheel_names.begin(),
                      wheel_names.end());
  n_joints_ = joint_names_.size();
  if (n_joints_ != kNumLegJoints + kNumWheels) {
    RCLCPP_ERROR(node_->get_logger(),
                 "QuadWheelController expected %d joints (12 leg + 4 wheel), "
                 "got %u",
                 kNumLegJoints + kNumWheels, n_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!load_leg_motor_limits(node_, torque_lims_, speed_lims_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!validate_motor_limits_3(node_, torque_lims_, speed_lims_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  load_wheel_motor_limits(wheel_torque_lims_, wheel_speed_lims_);

  for (unsigned int i = 0; i < n_joints_; ++i) {
    const auto& joint_name = joint_names_[i];
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
    if (!joint_urdf) {
      RCLCPP_ERROR(node_->get_logger(), "Could not find joint '%s' in urdf",
                   joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_urdfs_.push_back(joint_urdf);
  }

  commands_buffer_.writeFromNonRT(BufferType(kNumLegs));

  std::string joint_command_topic;
  node_->get_parameter("topics.control.joint_command", joint_command_topic);
  std::string ns = node_->get_namespace();
  joint_command_topic = ns + "/" + joint_command_topic;
  RCLCPP_INFO(node_->get_logger(),
              "QuadWheelController joint command topic: '%s'",
              joint_command_topic.c_str());

  sub_command_ = node_->create_subscription<quad_msgs::msg::LegCommandArray>(
      joint_command_topic, 1,
      std::bind(&QuadWheelController::commandCB, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "QuadWheelController configured (12 leg + 4 wheel joints)");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadWheelController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joint_cmd_handles_.clear();
  joint_pos_handles_.clear();
  joint_vel_handles_.clear();
  for (const auto& joint_name : joint_names_) {
    auto& cmd_handle = get_interface(command_interfaces_, joint_name, "effort");
    joint_cmd_handles_.push_back(std::move(cmd_handle));

    auto& pos_handle = get_interface(state_interfaces_, joint_name, "position");
    joint_pos_handles_.push_back(std::move(pos_handle));

    auto& vel_handle = get_interface(state_interfaces_, joint_name, "velocity");
    joint_vel_handles_.push_back(std::move(vel_handle));
  }
  RCLCPP_INFO(node_->get_logger(), "QuadWheelController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadWheelController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  BufferType& commands = *commands_buffer_.readFromRT();

  // Write a torque command to a joint handle, warning if the (nodiscard)
  // set_value fails so a dropped command doesn't pass silently.
  auto set_cmd = [this](unsigned int i, double torque) {
    if (!joint_cmd_handles_[i].set_value(torque)) {
      RCLCPP_WARN(node_->get_logger(),
                  "Failed to set Torque Command for Joint");
    }
  };

  // Before any command has been received, hold sitting pose with PD on
  // legs (matches QuadController's bootstrap behaviour) and zero wheel
  // effort so the robot doesn't roll away on activation.
  static bool first_command_received = false;
  if (!first_command_received) {
    if (commands.empty() || commands.front().motor_commands.empty()) {
      static const double sit_angles[3] = {0.0, 1.36, -2.65};
      static const double hold_kp = 40.0;
      static const double hold_kd = 2.0;
      for (unsigned int i = 0; i < n_joints_; ++i) {
        std::pair<int, int> ind = leg_map_[i];
        if (ind.second == 3) {
          // Wheel: zero torque while holding sit.
          set_cmd(i, 0.0);
          continue;
        }
        double target = sit_angles[ind.second];
        auto pos_opt = joint_pos_handles_[i].get_optional();
        auto vel_opt = joint_vel_handles_[i].get_optional();
        double pos = pos_opt.has_value() ? pos_opt.value() : 0.0;
        double vel = vel_opt.has_value() ? vel_opt.value() : 0.0;
        double torque = hold_kp * (target - pos) + hold_kd * (0.0 - vel);
        double torque_lim = torque_lims_[ind.second];
        torque = std::min(std::max(torque, -torque_lim), torque_lim);
        set_cmd(i, torque);
      }
      return controller_interface::return_type::OK;
    }
    first_command_received = true;
  }

  if (commands.empty() || commands.front().motor_commands.empty()) {
    return controller_interface::return_type::OK;
  }

  for (unsigned int i = 0; i < n_joints_; ++i) {
    std::pair<int, int> ind = leg_map_[i];
    const auto& leg_cmd = commands.at(ind.first);

    // Wheel slot is optional in the LegCommand. If a planner only fills
    // the 3 leg motors, the wheel coasts at zero effort.
    if (ind.second == 3 && leg_cmd.motor_commands.size() < 4) {
      set_cmd(i, 0.0);
      continue;
    }

    quad_msgs::msg::MotorCommand mc = leg_cmd.motor_commands.at(ind.second);
    double torque_ff = mc.torque_ff;

    // Wheel motors: kp ignored, velocity tracking only.
    if (ind.second == 3) {
      auto vel_opt = joint_vel_handles_[i].get_optional();
      double current_vel = vel_opt.has_value() ? vel_opt.value() : 0.0;
      double vel_error = mc.vel_setpoint - current_vel;
      double torque_command = mc.kd * vel_error + torque_ff;
      double torque_lim = wheel_torque_lims_.front();
      torque_command =
          std::min(std::max(torque_command, -torque_lim), torque_lim);
      set_cmd(i, torque_command);
      continue;
    }

    // Leg motors: PD + ff (mirrors QuadController exactly so go2 and
    // go2w simulate identically on the leg side).
    double command_position = mc.pos_setpoint;
    enforceJointLimits(command_position, i);
    auto pos_opt = joint_pos_handles_[i].get_optional();
    double current_position = pos_opt.has_value() ? pos_opt.value() : 0.0;
    double pos_error = 0.0;
    angles::shortest_angular_distance_with_large_limits(
        current_position, command_position, joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper, pos_error);

    auto vel_opt = joint_vel_handles_[i].get_optional();
    double current_vel = vel_opt.has_value() ? vel_opt.value() : 0.0;
    double vel_error = mc.vel_setpoint - current_vel;

    double torque_feedback = mc.kp * pos_error + mc.kd * vel_error;
    double torque_lim = torque_lims_[ind.second];
    double torque_command = std::min(
        std::max(torque_feedback + torque_ff, -torque_lim), torque_lim);
    set_cmd(i, torque_command);
  }
  return controller_interface::return_type::OK;
}

void QuadWheelController::commandCB(
    const quad_msgs::msg::LegCommandArray::SharedPtr msg) {
  commands_buffer_.writeFromNonRT(msg->leg_commands);
}

void QuadWheelController::enforceJointLimits(double& command,
                                             unsigned int index) {
  if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE ||
      joint_urdfs_[index]->type == urdf::Joint::PRISMATIC) {
    if (command > joint_urdfs_[index]->limits->upper) {
      command = joint_urdfs_[index]->limits->upper;
    } else if (command < joint_urdfs_[index]->limits->lower) {
      command = joint_urdfs_[index]->limits->lower;
    }
  }
}

}  // namespace effort_controllers

PLUGINLIB_EXPORT_CLASS(effort_controllers::QuadWheelController,
                       controller_interface::ControllerInterface)
