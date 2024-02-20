#ifndef UNDERBRUSH_INVERSE_DYNAMICS_H
#define UNDERBRUSH_INVERSE_DYNAMICS_H

#include <quad_msgs/BodyForceEstimate.h>
#include <robot_driver/controllers/leg_controller.h>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   InverseDynamicsController implements inverse dynamics logic. It should expose
   a constructor that does any initialization required and an update method
   called at some frequency.
*/
class UnderbrushInverseDynamicsController : public LegController {
 public:
  /**
   * @brief Constructor for InverseDynamicsController
   * @return Constructed object of type InverseDynamicsController
   */
  UnderbrushInverseDynamicsController();

  /**
   * @brief Update body force estimate
   * @param[in] msg current force estimates
   */
  void updateBodyForceEstimate(
      const quad_msgs::BodyForceEstimate::ConstPtr &msg);

  /**
   * @brief Set underbrush-specific gains and parameters
   * @param[in] retract_vel Retraction speed in rad/s when in contact
   * @param[in] tau_push Contact orwards push torque for distal link in N m
   * @param[in] tau_contact_start Threshold for contact initiation in N m
   * @param[in] tau_contact_end Threshold for contact ending in N m
   * @param[in] min_switch Minimum time between transitions in s
   * @param[in] t_down Time for foot to come down in s
   */
  void setUnderbrushParams(double retract_vel, double tau_push,
                           double tau_contact_start, double tau_contact_end,
                           double min_switch, double t_down, double t_up);

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

  /// Most recent body force estimate
  quad_msgs::BodyForceEstimate::ConstPtr last_body_force_estimate_msg_;

  /// Leg swing mode logic
  std::vector<int> force_mode_;
  std::vector<int> last_mode_;
  std::vector<double> t_switch_;
  std::vector<double> t_LO_;
  std::vector<double> t_TD_;

  /// Underbrush swing parameters
  double retract_vel_;
  double tau_push_;
  double tau_contact_start_;
  double tau_contact_end_;
  double min_switch_;
  double t_down_;
  double t_up_;
};

#endif  // UNDERBRUSH_INVERSE_DYNAMICS_H
