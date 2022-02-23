#include "robot_driver/robot_driver_utils.h"

namespace robot_driver_utils {

void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff, double kp, double kd,
  quad_msgs::MotorCommand &msg) {

  // Load commands
  msg.pos_setpoint = pos_setpoint;
  msg.vel_setpoint = vel_setpoint;
  msg.torque_ff = ff;
  msg.kp = kp;
  msg.kd = kd;

  // Compute diagnostics
  // loadMotorCommandMsg(leg_idx, joint_idx, pos_act, vel_act, torque_limits, msg);
}

// void loadMotorCommandMsg(int leg_idx, int joint_idx, double pos_act, double vel_act,
//   const Eigen::Vector3d &torque_limits, quad_msgs::MotorCommand &msg) {

//   // Compute diagnostics
//   msg.pos_component = msg.kp*(msg.pos_setpoint - pos_act);
//   msg.vel_component = msg.kd*(msg.vel_setpoint - vel_act);
//   msg.fb_component = msg.pos_component + msg.vel_component;
//   msg.effort = msg.fb_component + msg.torque_ff;
//   msg.fb_ratio = abs(msg.fb_component)/(abs(msg.fb_component) + abs(msg.torque_ff)); 

//   if (abs(msg.torque_ff) >= torque_limits[joint_idx]) {
//     ROS_WARN("Leg %d motor %d: ff effort = %5.3f Nm exceeds threshold of %5.3f Nm", 
//       leg_idx,joint_idx,msg.torque_ff, torque_limits[joint_idx]);

//   }      
//   if (abs(msg.effort) >= torque_limits[joint_idx]) {
//     ROS_WARN("Leg %d motor %d: total effort = %5.3f Nm exceeds threshold of %5.3f Nm", 
//       leg_idx,joint_idx,msg.effort, torque_limits[joint_idx]);
//     msg.effort = std::min(std::max(msg.effort, -torque_limits[joint_idx]), torque_limits[joint_idx]);
//   }
// }


}