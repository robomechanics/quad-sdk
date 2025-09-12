#include "gazebo_plugins/controller_plugin.hpp"

#include <angles/angles.h>

#include <pluginlib/class_list_macros.hpp>

namespace effort_controllers {

/**
 * \brief Forward command controller for a set of effort controlled joints
 * (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
QuadController::QuadController() { }

QuadController::~QuadController() { sub_command_.reset(); }

controller_interface::InterfaceConfiguration QuadController::command_interface_configuration() const {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names_) {
    interfaces.push_back(joint + "/effort");
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaces
  };
}

controller_interface::InterfaceConfiguration QuadController::state_interface_configuration() const {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names_) {
    interfaces.push_back(joint + "/position");
    interfaces.push_back(joint + "/velocity");
  }
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    interfaces
  };
}

controller_interface::CallbackReturn QuadController::on_deactivate(const rclcpp_lifecycle::State &) {
  sub_command_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadController::on_init(){
  // Setup joint map
  leg_map_[0] = std::make_pair(0, 1);   // hip0
  leg_map_[1] = std::make_pair(0, 2);   // knee0
  leg_map_[2] = std::make_pair(1, 1);   // hip1
  leg_map_[3] = std::make_pair(1, 2);   // knee1
  leg_map_[4] = std::make_pair(2, 1);   // hip2
  leg_map_[5] = std::make_pair(2, 2);   // knee2
  leg_map_[6] = std::make_pair(3, 1);   // hip3
  leg_map_[7] = std::make_pair(3, 2);   // knee3
  leg_map_[8] = std::make_pair(0, 0);   // abd0
  leg_map_[9] = std::make_pair(1, 0);   // abd1
  leg_map_[10] = std::make_pair(2, 0);  // abd2
  leg_map_[11] = std::make_pair(3, 0);  // abd3

  // Torque saturation (could change to linear model in future)
  torque_lims_ = {21, 21, 32};
  speed_lims_ = {37.7, 37.7, 25.1};

  node_ = get_node();
  node_->declare_parameter<std::vector<std::string>>("joints");
  node_->declare_parameter<std::string>("topics.control.joint_command");
  node_->declare_parameter<std::vector<double>>("torque_lims", {21.0, 21.0, 32.0});
  node_->declare_parameter<std::vector<double>>("speed_lims", {37.7, 37.7, 25.1});

  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn QuadController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){

    std::string urdf_string = get_robot_description();
    // RCLCPP_INFO(node_->get_logger(), "URDF: %s", urdf_string.c_str());

    urdf::Model urdf;
    if (!urdf.initString(urdf_string)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse urdf file");
        return controller_interface::CallbackReturn::ERROR;
    }

    std::string param_name = "joints";
    if (!node_->get_parameter(param_name, joint_names_)) {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Failed to get parameter '" << param_name
                                        << "' (namespace: " << node_->get_namespace() << ")");
    return controller_interface::CallbackReturn::ERROR;
    }
    std::string torque_param = "torque_lims";
    if (!node_->get_parameter(torque_param, torque_lims_)) {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Failed to get parameter '" << torque_param
                                        << "' (namespace: " << node_->get_namespace() << ")");
        return controller_interface::CallbackReturn::ERROR;
    }

    std::string speed_param = "speed_lims";
    if (!node_->get_parameter(speed_param, speed_lims_)) {
        RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Failed to get parameter '" << speed_param
                                        << "' (namespace: " << node_->get_namespace() << ")");
        return controller_interface::CallbackReturn::ERROR;
    }
    // RCLCPP_INFO(node_->get_logger(), "Torque Limits for each joint:");
    // for (size_t i = 0; i < joint_names_.size(); ++i) {
    //     if (i < torque_lims_.size()) {
    //         RCLCPP_INFO(node_->get_logger(), "  %s: %.3f Nm", joint_names_[i].c_str(), torque_lims_[i]);
    //     } else {
    //         RCLCPP_WARN(node_->get_logger(), "  %s: No torque limit specified!", joint_names_[i].c_str());
    //     }
    // }

    n_joints_ = joint_names_.size();
    if (n_joints_ == 0){
        RCLCPP_ERROR(node_->get_logger(), "List of joint names is empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    for (unsigned int i = 0; i < n_joints_; i++) {
        const auto& joint_name = joint_names_[i];

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf) {
            RCLCPP_ERROR(node_->get_logger(), "Could not find joint '%s' in urdf", joint_name.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        joint_urdfs_.push_back(joint_urdf);
    }

    int num_legs = 4;
    commands_buffer_.writeFromNonRT(BufferType(num_legs));

    std::string joint_command_topic;
    node_->get_parameter("topics.control.joint_command", joint_command_topic);
    // Necessary to override the topic naming for multi-robot use cases. Gazebo
    // plugins do not automatically inherit namespace
    std::string ns = node_->get_namespace();
    std::string private_ns = "joint_controller";
    // ns.resize(ns.size() - private_ns.size());
    joint_command_topic = ns + "/" + joint_command_topic;
    RCLCPP_INFO(node_->get_logger(), "Joint Command Topic 9 (simulated): '%s'", joint_command_topic.c_str());


    sub_command_ = node_->create_subscription<quad_msgs::msg::LegCommandArray>(joint_command_topic, 1, std::bind(&QuadController::commandCB, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "QuadController configured sucessfully");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QuadController::on_activate(const rclcpp_lifecycle::State& /* previous_state*/){
    joint_cmd_handles_.clear();
    joint_pos_handles_.clear();
    joint_vel_handles_.clear();
    // for (const auto &iface : command_interfaces_) {
    //     RCLCPP_INFO(node_->get_logger(), "Checking Interfaces");
    //     RCLCPP_INFO(node_->get_logger(), "Available interface: %s/%s",
    //         iface.get_name().c_str(), iface.get_interface_name().c_str());
    // }
    for (const auto& joint_name : joint_names_){
        // auto cmd_handle = command_interfaces_.at(joint_name + "/effort");
        // joint_cmd_handles_.push_back(hardware_interface::LoanedCommandInterface(cmd_handle));

        // auto pos_handle = state_interfaces_.at(joint_name + "/position");
        // joint_pos_handles_.push_back(hardware_interface::LoanedCommandInterface(pos_handle));

        // auto vel_handle = state_interfaces_.at(joint_name + "/velocity");
        // joint_vel_handles_.push_back(hardware_interface::LoanedCommandInterface(vel_handle));
        auto &cmd_handle = get_interface(command_interfaces_, joint_name, "effort");
        joint_cmd_handles_.push_back(std::move(cmd_handle));

        auto &pos_handle = get_interface(state_interfaces_, joint_name, "position");
        joint_pos_handles_.push_back(std::move(pos_handle));

        auto &vel_handle = get_interface(state_interfaces_, joint_name, "velocity");
        joint_vel_handles_.push_back(std::move(vel_handle));
    }
    RCLCPP_INFO(node_->get_logger(), "Successfully Activated QuadController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type QuadController::update(const rclcpp::Time &time, const rclcpp::Duration &period){
     BufferType& commands = *commands_buffer_.readFromRT();

  // Check if message is populated
  if (commands.empty() || commands.front().motor_commands.empty()) {
    return controller_interface::return_type::OK;
  }

  for (unsigned int i = 0; i < n_joints_; i++) {
    std::pair<int, int> ind = leg_map_[i];
    quad_msgs::msg::MotorCommand motor_command =
        commands.at(ind.first).motor_commands.at(ind.second);

    // Collect feedforward torque
    double torque_ff = motor_command.torque_ff;

    // Compute position error
    double command_position = motor_command.pos_setpoint;
    enforceJointLimits(command_position, i);
    // double current_position = joints_.at(i).getPosition();
    // double current_position = joint_pos_handles_[i].get_value(); // get_value deprecated in next ros release
    auto pose_opt = joint_pos_handles_[i].get_optional();
    double current_position = 0.0;
    if (pose_opt.has_value()){
        current_position = pose_opt.value();
    }
    else{
        RCLCPP_WARN(node_->get_logger(), "Missing Joint Pose Message");
    }
    double kp = motor_command.kp;
    double pos_error;
    angles::shortest_angular_distance_with_large_limits(
        current_position, command_position, joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper, pos_error);

    // Compute velocity error
    // double current_vel = joints_.at(i).getVelocity();
    // double current_vel = joint_vel_handles_[i].get_value();
    auto vel_opt = joint_vel_handles_[i].get_optional();
    double current_vel = 0.0;
    if (vel_opt.has_value()){
        current_vel = vel_opt.value();
    }
    else{
        RCLCPP_WARN(node_->get_logger(), "Missing Joint Velocity Message");
    }
    double command_vel = motor_command.vel_setpoint;
    double vel_error = command_vel - current_vel;
    double kd = motor_command.kd;

    // Collect feedback
    double torque_feedback = kp * pos_error + kd * vel_error;
    double torque_lim = torque_lims_[ind.second];
    double motor_model_ub = torque_lims_[ind.second] *
                            (1.0 - current_vel / speed_lims_[ind.second]);
    double motor_model_lb = -torque_lims_[ind.second] *
                            (1.0 + current_vel / speed_lims_[ind.second]);
    double torque_command = std::min(
        std::max(torque_feedback + torque_ff, -torque_lim), torque_lim);
    bool apply_motor_model = false;
    torque_command =
        (apply_motor_model)
            ? std::min(std::max(torque_command, motor_model_lb), motor_model_ub)
            : torque_command;

    // Update joint torque
    // joints_.at(i).setCommand(torque_command);
    bool success = joint_cmd_handles_[i].set_value(torque_command);
    if (!success){
        RCLCPP_WARN(node_->get_logger(), "Failed to set Torque Command for Joint");
    }

  }
  return controller_interface::return_type::OK;
}

void QuadController::commandCB(const quad_msgs::msg::LegCommandArray::SharedPtr msg) {
  commands_buffer_.writeFromNonRT(msg->leg_commands);
}

void QuadController::enforceJointLimits(double& command, unsigned int index) {
  // Check that this joint has applicable limits
  if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE ||
      joint_urdfs_[index]->type == urdf::Joint::PRISMATIC) {
    // above upper limit
    if (command > joint_urdfs_[index]->limits->upper) {
      command = joint_urdfs_[index]->limits->upper;
    } else if (command <
               joint_urdfs_[index]->limits->lower) {  // below lower limit
      command = joint_urdfs_[index]->limits->lower;
    }
  }
}

}  // namespace effort_controllers

PLUGINLIB_EXPORT_CLASS(effort_controllers::QuadController,
                       controller_interface::ControllerInterface)
