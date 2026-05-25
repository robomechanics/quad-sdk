#ifndef QUAD_CONTROLLERS__SAFETY_CONTROLLER_HPP_
#define QUAD_CONTROLLERS__SAFETY_CONTROLLER_HPP_

#include <vector>

#include "quad_controllers/quad_controller_base.hpp"

namespace quad_controllers {

//! Holds every joint at pos = 0 with safety PD gains. Used for SAFETY.
//! computeCommand sets pos_setpoint = 0, vel = 0, torque_ff = 0, kp/kd from
//! params, for all four legs.
class SafetyController : public QuadControllerBase {
 protected:
  controller_interface::CallbackReturn onInitExtra() override;
  controller_interface::CallbackReturn onConfigureExtra() override;
  bool computeCommand(quad_msgs::msg::LegCommandArray& out) override;

 private:
  std::vector<double> kp_;
  std::vector<double> kd_;
};

}  // namespace quad_controllers

#endif  // QUAD_CONTROLLERS__SAFETY_CONTROLLER_HPP_
