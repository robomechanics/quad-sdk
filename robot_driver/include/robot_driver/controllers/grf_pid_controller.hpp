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
   * @return Constructed object of type GrfPidController
   */
  GrfPidController(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                   std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[out] leg_command_array_msg Command message after solving inverse
   * dynamics and including reference setpoints for each joint
   * @param[out] grf_array_msg GRF command message
   */
  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg);

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
