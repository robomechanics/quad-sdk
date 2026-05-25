#ifndef QUAD_CONTROLLERS__LEG_CONTROL_CONTROLLER_HPP_
#define QUAD_CONTROLLERS__LEG_CONTROL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "quad_msgs/msg/leg_command_array.hpp"
#include "quad_msgs/msg/robot_plan.hpp"
#include "quad_msgs/msg/robot_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "quad_controllers/laws/leg_controller.hpp"
#include "quad_controllers/quad_controller_base.hpp"
#include "quad_utils/quad_kd2.hpp"

namespace quad_controllers {

//! READY-mode controller: delegates to a selectable leg-control law
//! (inverse_dynamics / grf_pid / joint / underbrush / inertia_estimation /
//! learned). Runs the law against the latest robot state; if the law fails or
//! no state has arrived it outputs the stand fallback (stand_joint_angles with
//! stand_kp/stand_kd). Faithful port of ControlModeController's READY branch.
class LegControlController : public QuadControllerBase {
 protected:
  controller_interface::CallbackReturn onInitExtra() override;
  controller_interface::CallbackReturn onConfigureExtra() override;
  bool computeCommand(quad_msgs::msg::LegCommandArray& out) override;

 private:
  bool initLegController();

  // Parameters.
  std::string controller_id_{"inverse_dynamics"};
  std::vector<double> stance_kp_, stance_kd_, swing_kp_, swing_kd_;
  std::vector<double> swing_kp_cart_, swing_kd_cart_;
  std::vector<double> stand_kp_, stand_kd_;
  std::vector<double> stand_joint_angles_;
  double policy_inference_rate_{50.0};
  double cmd_vel_filter_const_{0.10};
  double cmd_vel_scale_{1.0};
  std::string model_path_;
  std::string robot_state_topic_;
  // Underbrush swing-law params (consumed only by the `underbrush` law).
  double underbrush_retract_vel_{15.0};
  double underbrush_tau_push_{2.0};
  double underbrush_tau_contact_start_{3.0};
  double underbrush_tau_contact_end_{3.0};
  double underbrush_min_switch_{0.1};
  double underbrush_t_down_{0.135};
  double underbrush_t_up_{0.04};

  // Kinematics + selected control law, both built on this controller's own
  // LifecycleNode -- no helper node.
  std::shared_ptr<quad_utils::QuadKD2> quadKD_;
  std::shared_ptr<LegController> leg_controller_;

  // Inputs.
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr local_plan_sub_;
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr single_joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  quad_msgs::msg::RobotState::SharedPtr last_robot_state_msg_;
  Eigen::VectorXd cmd_vel_{Eigen::VectorXd::Zero(6)};
};

}  // namespace quad_controllers

#endif  // QUAD_CONTROLLERS__LEG_CONTROL_CONTROLLER_HPP_
