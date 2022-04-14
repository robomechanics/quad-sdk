#ifndef ROBOT_DRIVER_UTILS_H
#define ROBOT_DRIVER_UTILS_H

#include <quad_msgs/MotorCommand.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>

namespace robot_driver_utils {

void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff,
                         double kp, double kd, quad_msgs::MotorCommand &msg);

}  // namespace robot_driver_utils

#endif  // ROBOT_DRIVER_UTILS_H
