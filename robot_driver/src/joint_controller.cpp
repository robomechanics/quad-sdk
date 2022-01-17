#include "robot_driver/joint_controller.h"

JointController::JointController() {
  leg_idx_ = 0;
  joint_idx_ = 0;
  joint_torque_ = 0.0;
  this->override_state_machine_ = true;
}

void JointController::updateSingleJointCommand(const geometry_msgs::Vector3::ConstPtr& msg) {
  leg_idx_ = (int)msg->x;
  joint_idx_ = (int)msg->y;
  joint_torque_ = msg->z;
}

bool JointController::computeLegCommandArray(
  const quad_msgs::RobotState &robot_state_msg,
  quad_msgs::LegCommandArray &leg_command_array_msg,
  quad_msgs::GRFArray &grf_array_msg)
{
  leg_command_array_msg.leg_commands.resize(num_feet_);

  for (int i = 0; i < num_feet_; ++i) {
    leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {

      int joint_idx = 3*i+j;

      double joint_torque_val = 0;

      if ((i == leg_idx_) && (j == joint_idx_)) {
        joint_torque_val = std::max(std::min(joint_torque_,5.0),-5.0);
        ROS_INFO_THROTTLE(0.2, "Leg %d, joint %d, cmd = %5.3f", i,j,joint_torque_val);
      } else {
        joint_torque_val = 0;
      }


      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0.0;
      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 0.0;
      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff = joint_torque_val;

      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = 0.0;
      leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = 0.0;

    }
  }
  return true;
  
}