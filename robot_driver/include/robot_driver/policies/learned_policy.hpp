#ifndef LEARNED_POLICY_H
#define LEARNED_POLICY_H

#include <rclcpp/rclcpp.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/ros_utils.hpp>

#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

//! Implements an abstract class for learned policies.
/*!
   LearnedPolicy provides an abstract learned policy class. It contains
   pure virtual methods for running inference and computing motor commands for each leg to be sent to
   the robot.
*/

class LearnedPolicy {
 public:
 /**
  * @brief Constructor for LearnedPolicy
  * @return Constructed object of type LearnedPolicy
  */
  LearnedPolicy(rclcpp::Node::SharedPtr node, std::string& robot_ns);

 /**
  * @brief Set the desired proportional and derivative gains for all legs
  * @param[in] kp Proportional gains
  * @param[in] kd Derivative Gains
  */
  virtualvoid init(double kp, double kd):

 protected:

  /// Shared Pointer to Launch Node
  rclcpp::Node::SharedPtr node_;

  std::string robot_ns_;
  
  const int num_feet_ = 4;


};
#endif //LEARNED_POLICY_H