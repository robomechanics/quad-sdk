#include "controller_plugin.h"

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
SpiritController::SpiritController() {
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
}
SpiritController::~SpiritController() {
  sub_command_.shutdown();
  tail_sub_command_.shutdown();
}

bool SpiritController::init(hardware_interface::EffortJointInterface* hw,
                            ros::NodeHandle& n) {
  // List of controlled joints
  std::string param_name = "joints";
  if (!n.getParam(param_name, joint_names_)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: "
                                            << n.getNamespace() << ").");
    return false;
  }
  n_joints_ = joint_names_.size();

  if (n_joints_ == 0) {
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }

  // Get URDF
  urdf::Model urdf;
  if (!urdf.initParam("robot_description")) {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  for (unsigned int i = 0; i < n_joints_; i++) {
    const auto& joint_name = joint_names_[i];

    try {
      joints_.push_back(hw->getHandle(joint_name));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }

    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
    if (!joint_urdf) {
      ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
      return false;
    }
    joint_urdfs_.push_back(joint_urdf);
  }

  int num_legs = 4;
  commands_buffer_.writeFromNonRT(BufferType(num_legs));

  std::string joint_command_topic;
  quad_utils::loadROSParam(n, "/topics/control/joint_command",
                           joint_command_topic);

  sub_command_ = n.subscribe<quad_msgs::LegCommandArray>(
      joint_command_topic, 1, &SpiritController::commandCB, this,
      ros::TransportHints().tcpNoDelay(true));

  int num_tail_motors = 2;
  tail_commands_buffer_.writeFromNonRT(TailBufferType(num_tail_motors));

  std::string tail_command_topic;
  quad_utils::loadROSParam(n, "/topics/control/tail_command",
                           tail_command_topic);

  tail_sub_command_ = n.subscribe<quad_msgs::LegCommand>(
      tail_command_topic, 1, &SpiritController::tailCommandCB, this,
      ros::TransportHints().tcpNoDelay(true));

  n.param<int>("/tail_controller/tail_type", tail_type_, 0);

  return true;
}

void SpiritController::update(const ros::Time& time,
                              const ros::Duration& period) {
  BufferType& commands = *commands_buffer_.readFromRT();
  TailBufferType& tail_commands = *tail_commands_buffer_.readFromRT();

  // Check if message is populated
  if (commands.empty() || commands.front().motor_commands.empty() ||
      (tail_type_ != NONE && tail_commands.empty())) {
    return;
  }

  if (tail_type_ != NONE) {
    for (unsigned int i = 12; i < 14; i++) {
      // Tail control
      quad_msgs::MotorCommand motor_command = tail_commands.at(i - 12);

      // Collect feedforward torque
      double torque_ff = motor_command.torque_ff;

      // Compute position error
      double command_position = motor_command.pos_setpoint;
      enforceJointLimits(command_position, i);
      double current_position = joints_.at(i).getPosition();
      double kp = motor_command.kp;
      double pos_error = command_position - current_position;

      // Compute velocity error
      double current_vel = joints_.at(i).getVelocity();
      double command_vel = motor_command.vel_setpoint;
      double vel_error = command_vel - current_vel;
      double kd = motor_command.kd;

      // Collect feedback
      double torque_feedback = kp * pos_error + kd * vel_error;
      // double torque_lim = torque_lims_[ind.second];
      double torque_command = torque_feedback + torque_ff;
      torque_command =
          std::max(std::min(torque_command, joint_urdfs_[i]->limits->effort),
                   -joint_urdfs_[i]->limits->effort);

      // std::cout << "Joint " << i << ": " << "FF Torque: " << torque_ff << "
      // FF Torque %: " << torque_ff/torque_command << " FB Torque: " <<
      // torque_feedback << " FB Torque %: " << torque_feedback/torque_command
      // << " Total Torque: " << torque_command << std::endl;

      // Update joint torque
      joints_.at(i).setCommand(torque_command);
    }
  }

  for (unsigned int i = 0; i < 12; i++) {
    std::pair<int, int> ind = leg_map_[i];
    quad_msgs::MotorCommand motor_command =
        commands.at(ind.first).motor_commands.at(ind.second);

    // Collect feedforward torque
    double torque_ff = motor_command.torque_ff;

    // Compute position error
    double command_position = motor_command.pos_setpoint;
    enforceJointLimits(command_position, i);
    double current_position = joints_.at(i).getPosition();
    double kp = motor_command.kp;
    double pos_error;
    angles::shortest_angular_distance_with_large_limits(
        current_position, command_position, joint_urdfs_[i]->limits->lower,
        joint_urdfs_[i]->limits->upper, pos_error);

    // Compute velocity error
    double current_vel = joints_.at(i).getVelocity();
    double command_vel = motor_command.vel_setpoint;
    double vel_error = command_vel - current_vel;
    double kd = motor_command.kd;

    // Collect feedback
    double torque_feedback = kp * pos_error + kd * vel_error;
    double torque_lim = torque_lims_[ind.second];
    double torque_command = std::min(
        std::max(torque_feedback + torque_ff, -torque_lim), torque_lim);

    // std::cout << "Joint " << i << ": " << "FF Torque: " << torque_ff << " FF
    // Torque %: " << torque_ff/torque_command << " FB Torque: " <<
    // torque_feedback << " FB Torque %: " << torque_feedback/torque_command <<
    // " Total Torque: " << torque_command << std::endl;

    // Update joint torque
    joints_.at(i).setCommand(torque_command);
  }
}

void SpiritController::commandCB(
    const quad_msgs::LegCommandArrayConstPtr& msg) {
  commands_buffer_.writeFromNonRT(msg->leg_commands);
}

void SpiritController::tailCommandCB(const quad_msgs::LegCommandConstPtr& msg) {
  tail_commands_buffer_.writeFromNonRT(msg->motor_commands);
}

void SpiritController::enforceJointLimits(double& command, unsigned int index) {
  // Check that this joint has applicable limits
  if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE ||
      joint_urdfs_[index]->type == urdf::Joint::PRISMATIC) {
    // above upper limnit
    if (command > joint_urdfs_[index]->limits->upper) {
      command = joint_urdfs_[index]->limits->upper;
    } else if (command < joint_urdfs_[index]->limits->lower) {
      // below lower limit
      command = joint_urdfs_[index]->limits->lower;
    }
  }
}

}  // namespace effort_controllers

PLUGINLIB_EXPORT_CLASS(effort_controllers::SpiritController,
                       controller_interface::ControllerBase)
