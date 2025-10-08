#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <geometry_msgs/msg/vector3.hpp>
#include <robot_driver/controllers/leg_controller.hpp>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   JointController implements inverse dynamics logic. It should expose a
   constructor that does any initialization required and an update method called
   at some frequency.
*/
class JointController : public LegController {
 public:
  /**
   * @brief Constructor for JointController
   * @param[in] node Shared pointer to rclcpp::Node
   * @param[in] robot_ns Robot Namespace
   * @return Constructed object of type JointController
   */
  JointController(rclcpp::Node::SharedPtr node, std::string &robot_ns);

  void updateSingleJointCommand(const geometry_msgs::msg::Vector3::SharedPtr &msg);

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[in] robot_state_msg Message including robot state
   * @param[in] leg_command_array_msg Command message after solving inverse
   * dynamics and including reference setpoints for each joint
   * @param[in] grf_array_msg GRF command message
   */
  bool computeLegCommandArray(const quad_msgs::msg::RobotState &robot_state_msg,
                              quad_msgs::msg::LegCommandArray &leg_command_array_msg,
                              quad_msgs::msg::GRFArray &grf_array_msg);

 private:
  /// Leg index for controlled joint
  int leg_idx_ = 0;

  /// Joint index for controlled joint
  int joint_idx_ = 0;

  /// Desired torque in Nm
  double joint_torque_ = 0.0;
};

#endif  // JOINT_CONTROLLER_H
