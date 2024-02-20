#ifndef INERTIA_ESTIMATION_CONTROLLER_H
#define INERTIA_ESTIMATION_CONTROLLER_H

#include <robot_driver/controllers/leg_controller.h>

//! Special-purpose inertia parameter estimation leg controller.
class InertiaEstimationController : public LegController {
 public:
  /**
   * @brief Constructor for InertiaEstimationController
   * @return Constructed object of type InertiaEstimationController
   */
  InertiaEstimationController();

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[in] robot_state_msg Message of the current robot state
   * @param[out] leg_command_array_msg Command message after solving inverse
   * dynamics and including reference setpoints for each joint
   * @param[out] grf_array_msg GRF command message
   */
  bool computeLegCommandArray(const quad_msgs::RobotState &robot_state_msg,
                              quad_msgs::LegCommandArray &leg_command_array_msg,
                              quad_msgs::GRFArray &grf_array_msg);

  /**
   * @brief Return the reference state used for current tracking
   * @return Reference state
   */
  inline quad_msgs::RobotState getReferenceState() { return ref_state_msg_; }

 private:
  /// Prior grf_array
  Eigen::VectorXd last_grf_array_;

  /// Reference state for tracking
  quad_msgs::RobotState ref_state_msg_;

  /// GRF exponential filter constant
  const double grf_exp_filter_const_ = 1.0;  // 1.0 = no filtering
};

#endif  // INERTIA_ESTIMATION_CONTROLLER
