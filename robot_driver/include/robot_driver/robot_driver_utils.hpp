#ifndef ROBOT_DRIVER_UTILS_H
#define ROBOT_DRIVER_UTILS_H

#include <quad_msgs/msg/motor_command.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/ros_utils.hpp>

namespace robot_driver_utils {

/**
 * @brief Fill a MotorCommand message from scalar command fields
 * @param[in] pos_setpoint Position setpoint
 * @param[in] vel_setpoint Velocity setpoint
 * @param[in] ff Feedforward torque
 * @param[in] kp Proportional gain
 * @param[in] kd Derivative gain
 * @param[out] msg MotorCommand message to populate
 */
void loadMotorCommandMsg(double pos_setpoint, double vel_setpoint, double ff,
                         double kp, double kd,
                         quad_msgs::msg::MotorCommand& msg);

}  // namespace robot_driver_utils

#endif  // ROBOT_DRIVER_UTILS_H
