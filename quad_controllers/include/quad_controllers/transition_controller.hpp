#ifndef QUAD_CONTROLLERS__TRANSITION_CONTROLLER_HPP_
#define QUAD_CONTROLLERS__TRANSITION_CONTROLLER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "quad_controllers/quad_controller_base.hpp"

namespace quad_controllers {

//! Linearly interpolates every joint from `from_angles` to `to_angles` over
//! `duration` seconds with PD gains. Used for SIT_TO_READY (from=sit, to=stand)
//! and READY_TO_SIT (from=stand, to=sit). The interpolation clock starts on
//! on_activate.
class TransitionController : public QuadControllerBase {
 public:
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State&) override;

 protected:
  controller_interface::CallbackReturn onInitExtra() override;
  controller_interface::CallbackReturn onConfigureExtra() override;
  bool computeCommand(quad_msgs::msg::LegCommandArray& out) override;

 private:
  std::vector<double> from_angles_;
  std::vector<double> to_angles_;
  double duration_{1.0};
  std::vector<double> kp_;
  std::vector<double> kd_;

  rclcpp::Time start_time_;
};

}  // namespace quad_controllers

#endif  // QUAD_CONTROLLERS__TRANSITION_CONTROLLER_HPP_
