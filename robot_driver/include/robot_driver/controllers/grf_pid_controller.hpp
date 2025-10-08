#ifndef GRF_PID_CONTROLLER
#define GRF_PID_CONTROLLER

#include <robot_driver/controllers/leg_controller.hpp>

//! Implements inverse dynamics as a controller within the ROS framework.
/*!
   GrfPidController implements inverse dynamics logic. It should expose a
   constructor that does any initialization required and an update method called
   at some frequency.
*/
class GrfPidController : public LegController {
 public:
  /**
   * @brief Constructor for GrfPidController
   * @param[in] node Shared pointer to rclcpp::Node
   * @param[in] robot_ns Robot Namespace
   * @return Constructed object of type GrfPidController
   */
  GrfPidController(rclcpp::Node::SharedPtr node, std::string& robot_ns);

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
  /// Desired position
  Eigen::Vector3d pos_des_;

  /// Desired orientation
  Eigen::Vector3d ang_des_;

  /// Position error integral
  Eigen::Vector3d pos_error_int_;

  /// Orientation error integral
  Eigen::Vector3d ang_error_int_;

  /// Timekeeping variable for integral term
  rclcpp::Time t_old_;
};

#endif  // GRF_PID_CONTROLLER
