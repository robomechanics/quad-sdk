#pragma once

#include <quad_msgs/msg/motor_command.hpp>
#include "robot_driver/controllers/underbrush_torque_observation.hpp"

namespace underbrush {

// Convert one policy PD target to the actual feedforward field sent to Unitree.
// Call at the motor update rate with fresh state, not only at actor inference.
// Keep position/velocity targets for logging; zero gains disable onboard PD.
// On invalid input, leave the command unchanged and let the driver enter safety.
inline bool makeClippedEffortCommand(quad_msgs::msg::MotorCommand& cmd,
                                     double position, double velocity,
                                     double absolute_limit) {
  if (!std::isfinite(position) || !std::isfinite(velocity) ||
      !std::isfinite(cmd.pos_setpoint) || !std::isfinite(cmd.vel_setpoint) ||
      !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd) ||
      !std::isfinite(cmd.torque_ff) || !std::isfinite(absolute_limit) ||
      cmd.kp < 0 || cmd.kd < 0 || absolute_limit <= 0) {
    return false;
  }
  const double position_effort = cmd.kp * (cmd.pos_setpoint - position);
  const double velocity_effort = cmd.kd * (cmd.vel_setpoint - velocity);
  const double feedback = position_effort + velocity_effort;
  const double requested = feedback + cmd.torque_ff;
  if (!std::isfinite(requested)) return false;

  const double effort = std::clamp(clipGo2HVEffort(requested, velocity),
                                   -absolute_limit, absolute_limit);
  const double denominator = std::abs(feedback) + std::abs(cmd.torque_ff);
  cmd.pos_component = position_effort;
  cmd.vel_component = velocity_effort;
  cmd.fb_component = feedback;
  cmd.fb_ratio = denominator > 0 ? std::abs(feedback) / denominator : 0.0;
  cmd.effort = effort;
  cmd.torque_ff = effort;
  cmd.kp = 0.0f;
  cmd.kd = 0.0f;
  return true;
}

}  // namespace underbrush
