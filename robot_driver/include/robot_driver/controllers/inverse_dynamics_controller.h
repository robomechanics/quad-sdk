#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

#include <robot_driver/controllers/leg_controller.h>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   InverseDynamicsController implements inverse dynamics logic. It should expose
   a constructor that does any initialization required and an update method
   called at some frequency.
*/
class InverseDynamicsController : public LegController {
 public:
  /**
   * @brief Constructor for InverseDynamicsController
   * @return Constructed object of type InverseDynamicsController
   */
  InverseDynamicsController();

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

 private:
  /// Prior grf_array
  Eigen::VectorXd last_grf_array_;

  /// GRF exponential filter constant
  const double grf_exp_filter_const_ = 1.0;  // 1.0 = no filtering
};

#endif  // INVERSE_DYNAMICS_H
