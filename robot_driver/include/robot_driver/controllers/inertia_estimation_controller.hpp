#ifndef INERTIA_ESTIMATION_CONTROLLER_H
#define INERTIA_ESTIMATION_CONTROLLER_H

#include <robot_driver/controllers/leg_controller.hpp>

//! Special-purpose inertia parameter estimation leg controller.
class InertiaEstimationController : public LegController {
 public:
  /**
   * @brief Constructor for InertiaEstimationController
   * @return Constructed object of type InertiaEstimationController
   */
  InertiaEstimationController(rclcpp::Node::SharedPtr node,
                              const std::string& robot_ns,
                              std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[in] robot_state_msg Message of the current robot state
   * @param[out] leg_command_array_msg Command message after solving inverse
   * dynamics and including reference setpoints for each joint
   * @param[out] grf_array_msg GRF command message
   */
  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg);

  /**
   * @brief Return the reference state used for current tracking
   * @return Reference state
   */
  inline quad_msgs::msg::RobotState getReferenceState() {
    return ref_state_msg_;
  }

 private:
  /// Prior grf_array
  Eigen::VectorXd last_grf_array_;

  /// Reference state for tracking
  quad_msgs::msg::RobotState ref_state_msg_;

  /// GRF exponential filter constant
  const double grf_exp_filter_const_ = 1.0;  // 1.0 = no filtering

  /// Per-leg per-joint (abad, hip, knee) URDF-vs-controller convention
  /// coefficients loaded lazily from ROS params on first compute call.
  /// Applied as `q_wire = q_ctrl * sign + offset` so the same Spirit40-tuned
  /// excitation setpoint below works on any robot whose yaml (leg_<i>.joints.
  /// {abad|hip|knee}.{sign,offset}) is populated. Defaults (1, 0) preserve
  /// Spirit40 behavior for legacy configs that omit these fields.
  bool conv_loaded_ = false;
  std::vector<std::vector<double>> joint_sign_;
  std::vector<std::vector<double>> joint_offset_;

  /// Sys-ID collection knobs, loaded lazily with the convention coefficients.
  /// target_leg: 0..3 flails only that leg (quad order 0=FL 1=BL 2=FR 3=BR),
  /// -1 flails all four. time_scale scales the excitation clock: 1.0 = the
  /// hardware-verified envelope, 0.5 = same amplitudes at half velocity /
  /// quarter acceleration (slow pass for Coulomb-vs-viscous separation).
  int target_leg_ = 0;
  double time_scale_ = 1.0;

  /// isolate_joint: -1 = off (all 3 joints flail, legacy behavior);
  /// 0=abad 1=hip 2=knee = flail ONLY that joint of the target leg and hold
  /// the other two at the pose latched when isolation starts. Purpose:
  /// abad's tau_est is dominated by config-dependent gravity/inertia
  /// coupling from the hip/knee flail (July fit R2 0.11); freezing them
  /// makes the leg a rigid pendulum in the roll plane so the single-axis
  /// gravity regressor is exact.
  int isolate_joint_ = -1;
  std::vector<double> isolate_hold_pose_;  // wire-space, latched once
};

#endif  // INERTIA_ESTIMATION_CONTROLLER
