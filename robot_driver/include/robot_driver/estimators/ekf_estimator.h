#ifndef EKF_H
#define EKF_H

#include <robot_driver/estimators/state_estimator.h>

//! Implements Extended Kalman Filter as an estimator within the ROS framework.
class EKFEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFEstimator
   * @return Constructed object of type EKFEstimator
   */
  EKFEstimator();

  /**
   * @brief Initialize EKF
   * @param[in] nh Node Handler to load parameters from yaml file
   */
  void init(ros::NodeHandle& nh);

  /**
   * @brief Perform EKF update once
   * @param[out] last_robot_state_msg_
   */
  bool updateOnce(quad_msgs::RobotState& last_robot_state_msg_);

 private:
  /// Nodehandle to get param
  ros::NodeHandle nh_;
};

#endif  // EKF_H
