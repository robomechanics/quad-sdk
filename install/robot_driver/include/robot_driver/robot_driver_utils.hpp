#ifndef ROBOT_DRIVER_UTILS_H
#define ROBOT_DRIVER_UTILS_H

#include <quad_msgs/msg/motor_command.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/ros_utils.hpp>

namespace robot_driver_utils {

void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff,
                         double kp, double kd,
                         quad_msgs::msg::MotorCommand& msg);

}  // namespace robot_driver_utils

#endif  // ROBOT_DRIVER_UTILS_H
