#ifndef QUAD_CONTROLLERS__POSE_CONTROLLER_HPP_
#define QUAD_CONTROLLERS__POSE_CONTROLLER_HPP_

#include <vector>

#include "quad_controllers/quad_controller_base.hpp"

namespace quad_controllers {

//! Holds every leg at a fixed joint pose with PD gains. Used for SIT.
//! computeCommand sets pos_setpoint = joint_angles[j], vel = 0, torque_ff = 0,
//! kp/kd from params, for all four legs.
class PoseController : public QuadControllerBase {
 protected:
  controller_interface::CallbackReturn onInitExtra() override;
  controller_interface::CallbackReturn onConfigureExtra() override;
  bool computeCommand(quad_msgs::msg::LegCommandArray& out) override;

 private:
  std::vector<double> joint_angles_;  // 3 per-leg: Abd, Hip, Knee
  std::vector<double> kp_;
  std::vector<double> kd_;
};

}  // namespace quad_controllers

#endif  // QUAD_CONTROLLERS__POSE_CONTROLLER_HPP_
