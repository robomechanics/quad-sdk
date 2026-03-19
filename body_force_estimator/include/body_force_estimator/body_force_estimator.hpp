#ifndef BODY_FORCE_ESTIMATOR_H
#define BODY_FORCE_ESTIMATOR_H

#include <quad_msgs/msg/body_force_estimate.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>

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
  BodyForceEstimator(rclcpp::Node::SharedPtr node);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

  /**
   * @brief Callback function to handle new state estimates
   * @param[in] Robot state message contining position and velocity for each
   * joint and robot body
   */
  void robotStateCallback(const quad_msgs::msg::RobotState::SharedPtr msg);

  /**
   * @brief Callback function to handle new local plan (states and GRFs)
   * @param[in] msg input message contining the local plan
   */
  void localPlanCallback(const quad_msgs::msg::RobotPlan::SharedPtr msg);

  /**
   * @brief Compute the momentum observer external force estimation update.
   */
  void update();

  /**
   * @brief Publish body force force estimates
   */
  void publishBodyForce();

  /// ROS subscriber for the robot state
  rclcpp::Subscription<quad_msgs::msg::RobotState>::SharedPtr robot_state_sub_;

  /// ROS subscriber for local plan
  rclcpp::Subscription<quad_msgs::msg::RobotPlan>::SharedPtr local_plan_sub_;

  /// ROS publisher for body force force estimates
  rclcpp::Publisher<quad_msgs::msg::BodyForceEstimate>::SharedPtr
      body_force_pub_;

  /// ROS publisher for toe force estimates
  rclcpp::Publisher<quad_msgs::msg::GRFArray>::SharedPtr toe_force_pub_;

  /// Nodehandle to pub to and sub from
  rclcpp::Node::SharedPtr node_;

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
  quad_msgs::msg::RobotPlan::SharedPtr last_local_plan_msg_;

  /// Previous foot state
  quad_msgs::msg::MultiFootState past_feet_state_;

  // Robot state estimate
  quad_msgs::msg::RobotState::SharedPtr last_state_msg_;
};

#endif  // BODY_FORCE_ESTIMATOR_H
