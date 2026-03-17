#ifndef EKF_H
#define EKF_H

#include <robot_driver/estimators/state_estimator.hpp>

//! Implements Extended Kalman Filter as an estimator within the ROS framework.
class EKFEstimator : public StateEstimator {
 public:
  /**
   * @brief Constructor for EKFEstimator
   * @return Constructed object of type EKFEstimator
   */
  EKFEstimator(rclcpp::Node::SharedPtr node, const std::string& robot_ns, std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Initialize EKF
   * @param[in] nh Node Handler to load parameters from yaml file
   */
  void init() override;

  /**
   * @brief Perform EKF update once
   * @param[out] last_robot_state_msg_
   */
  bool updateOnce(quad_msgs::msg::RobotState& last_robot_state_msg_) override;

 private:

};

#endif  // EKF_H
