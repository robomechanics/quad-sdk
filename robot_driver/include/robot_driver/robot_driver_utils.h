#ifndef ROBOT_DRIVER_UTILS_H
#define ROBOT_DRIVER_UTILS_H

#include <quad_utils/ros_utils.h>
#include <quad_utils/math_utils.h>

#include <quad_msgs/MotorCommand.h>

namespace robot_driver_utils {

void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff, double kp, double kd,
    quad_msgs::MotorCommand &msg);

// void loadMotorCommandMsg(int leg_idx, int joint_idx, double pos_act, double vel_act,
//   const Eigen::Vector3d &torque_limits, quad_msgs::MotorCommand &msg);

}

#endif // ROBOT_DRIVER_UTILS_H