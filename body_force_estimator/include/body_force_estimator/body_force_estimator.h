#ifndef BODY_FORCE_ESTIMATOR_H
#define BODY_FORCE_ESTIMATOR_H

#include <quad_msgs/BodyForceEstimate.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

//! Estimates body contact forces
/*!
   BodyForceEstimator is a container for all logic used in estimating force from
   contacts distrbuted across all links of the robot. It requires robot state
   estimates and motor commands and exposes an update method.
*/
class BodyForceEstimator {
 public:
  /**
   * @brief Constructor for BodyForceEstimator Class
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type BodyForceEstimator
   */
  BodyForceEstimator(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

  /**
   * @brief Callback function to handle new state estimates
   * @param[in] Robot state message contining position and velocity for each
   * joint and robot body
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new local plan (states and GRFs)
   * @param[in] msg input message contining the local plan
   */
  void localPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg);

  /**
   * @brief Compute the momentum observer external force estimation update.
   */
  void update();

  /**
   * @brief Publish body force force estimates
   */
  void publishBodyForce();

  /// ROS subscriber for the robot state
  ros::Subscriber robot_state_sub_;

  /// ROS subscriber for local plan
  ros::Subscriber local_plan_sub_;

  /// ROS publisher for body force force estimates
  ros::Publisher body_force_pub_;

  /// ROS publisher for toe force estimates
  ros::Publisher toe_force_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Momentum observer gain
  double K_O_;

  /// Momentum observer cancel friction or not
  int cancel_friction_;

 private:
  /// External torque estimate
  double r_mom[12];

  /// Momentum estimate
  double p_hat[12];

  /// Most recent local plan
  quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

  /// Previous foot state
  quad_msgs::MultiFootState past_feet_state_;

  // Robot state estimate
  quad_msgs::RobotState::ConstPtr last_state_msg_;
};

#endif  // BODY_FORCE_ESTIMATOR_H
