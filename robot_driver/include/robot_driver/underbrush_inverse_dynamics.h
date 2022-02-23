#ifndef UNDERBRUSH_INVERSE_DYNAMICS_H
#define UNDERBRUSH_INVERSE_DYNAMICS_H

#include <robot_driver/leg_controller_template.h>
#include <quad_msgs/BodyForceEstimate.h>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   InverseDynamicsController implements inverse dynamics logic. It should expose a constructor that does any initialization required and an update method called at some frequency.
*/
class UnderbrushInverseDynamicsController : public LegControllerTemplate {
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
    void updateBodyForceEstimate(const quad_msgs::BodyForceEstimate::ConstPtr& msg);

    /**
     * @brief Compute the leg command array message for a given current state and reference plan
     * @param[in] robot_state_msg Message of the current robot state
     * @param[out] leg_command_array_msg Command message after solving inverse dynamics and including reference setpoints for each joint
     * @param[out] grf_array_msg GRF command message
     */
    bool computeLegCommandArray(
      const quad_msgs::RobotState &robot_state_msg,
      quad_msgs::LegCommandArray &leg_command_array_msg,
      quad_msgs::GRFArray &grf_array_msg);

private:

  /// Prior grf_array
  Eigen::VectorXd last_grf_array_;

  /// GRF exponential filter constant
  const double grf_exp_filter_const_ = 1.0; // 1.0 = no filtering

  /// Most recent body force estimate
  quad_msgs::BodyForceEstimate::ConstPtr last_body_force_estimate_msg_;

  std::vector<int> force_mode_;
  std::vector<double> t_switch_;
  std::vector<double> t_LO_;
  std::vector<double> t_TD_;

};


#endif // UNDERBRUSH_INVERSE_DYNAMICS_H
