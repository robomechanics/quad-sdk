#include "robot_driver/robot_driver_utils.hpp"

namespace robot_driver_utils {

void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff,
                         double kp, double kd,
                         quad_msgs::msg::MotorCommand& msg) {
  // Load commands
  msg.pos_setpoint = pos_setpoint;
  msg.vel_setpoint = vel_setpoint;
  msg.torque_ff = ff;
  msg.kp = kp;
  msg.kd = kd;
}

}  // namespace robot_driver_utils
