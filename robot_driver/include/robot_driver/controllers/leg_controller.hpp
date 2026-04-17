#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

// #include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <quad_msgs/msg/grf_array.hpp>
#include <quad_msgs/msg/leg_command.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <quad_msgs/msg/motor_command.hpp>
#include <quad_msgs/msg/multi_foot_plan_continuous.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_utils/math_utils.hpp>
#include <quad_utils/quad_kd2.hpp>
#include <quad_utils/ros_utils.hpp>
#include <robot_driver/hardware_interfaces/spirit_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <cmath>
#include <eigen3/Eigen/Eigen>
#define MATH_PI 3.141592

//! Implements an abstract class for leg controllers.
/*!
   LegController provides an abstract leg controller class. It contains
   pure virtual methods for computing motor commands for each leg to be sent to
   the robot.
*/
class LegController {
 public:
  /**
   * @brief Constructor for LegController
   * @return Constructed object of type LegController
   */
  LegController(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Set the desired proportional and derivative gains for all legs
   * @param[in] kp Proportional gains
   * @param[in] kd Derivative gains
   */
  virtual void init(double kp, double kd);

  /**
   * @brief Set the desired proportional and derivative gains for each leg
   * @param[in] kp Proportional gains
   * @param[in] kd Derivative gains
   */
  virtual void init(const std::vector<double>& kp,
                    const std::vector<double>& kd);

  /**
   * @brief Set the desired stance and swing proportional and derivative gains
   * @param[in] stance_kp Stance phase proportional gains
   * @param[in] stance_kd Stance phase derivative gains
   * @param[in] swing_kp Swing phase proportional gains
   * @param[in] swing_kd Swing phase derivative gains
   * @param[in] swing_kp_cart Cartesian Swing phase proportional gains
   * @param[in] swing_kd_cart Cartesian Swing phase derivative gains
   */
  virtual void init(const std::vector<double>& stance_kp,
                    const std::vector<double>& stance_kd,
                    const std::vector<double>& swing_kp,
                    const std::vector<double>& swing_kd,
                    const std::vector<double>& swing_kp_cart,
                    const std::vector<double>& swing_kd_cart);

  /**
   * @brief Set the desired stance and swing proportional and derivative gains
   * @param[in] stance_kp Stance phase proportional gains
   * @param[in] stance_kd Stance phase derivative gains
   * @param[in] swing_kp Swing phase proportional gains
   * @param[in] swing_kd Swing phase derivative gains
   * @param[in] swing_kp_cart Cartesian Swing phase proportional gains
   * @param[in] swing_kd_cart Cartesian Swing phase derivative gains
   * @param[in] model_path Absolute Path to ONNX Model Weights
   * @param[in] policy_inference_rate Rate (Hz) at which learned policy runs
   *            inference; PD tracking runs at the main loop rate
   */
  virtual void init(const std::vector<double>& stance_kp,
                    const std::vector<double>& stance_kd,
                    const std::vector<double>& swing_kp,
                    const std::vector<double>& swing_kd,
                    const std::vector<double>& swing_kp_cart,
                    const std::vector<double>& swing_kd_cart,
                    const std::string& model_path,
                    double policy_inference_rate = 50.0);

  /**
   * @brief Set gains, model path, inference rate, and stand joint angles
   * @param[in] stance_kp Stance phase proportional gains
   * @param[in] stance_kd Stance phase derivative gains
   * @param[in] swing_kp Swing phase proportional gains
   * @param[in] swing_kd Swing phase derivative gains
   * @param[in] swing_kp_cart Cartesian Swing phase proportional gains
   * @param[in] swing_kd_cart Cartesian Swing phase derivative gains
   * @param[in] model_path Absolute Path to ONNX Model Weights
   * @param[in] policy_inference_rate Rate (Hz) at which learned policy runs
   * @param[in] stand_joint_angles Default standing joint angles
   */
  virtual void init(const std::vector<double>& stance_kp,
                    const std::vector<double>& stance_kd,
                    const std::vector<double>& swing_kp,
                    const std::vector<double>& swing_kd,
                    const std::vector<double>& swing_kp_cart,
                    const std::vector<double>& swing_kd_cart,
                    const std::string& model_path,
                    double policy_inference_rate,
                    const std::vector<double>& stand_joint_angles);

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[in] local_plan_msg Message of the local referance plan
   */
  void updateLocalPlanMsg(quad_msgs::msg::RobotPlan::SharedPtr msg,
                          const rclcpp::Time& t_msg);

  /**
   * @brief Compute the leg command array message
   */
  virtual bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg) = 0;

  inline bool overrideStateMachine() { return override_state_machine_; }

 protected:
  /// Shared Pointer to Launch Node
  rclcpp::Node::SharedPtr node_;

  /// Robot Namespace
  std::string robot_ns_;

  /// Number of feet
  const int num_feet_ = 4;

  /// QuadKD class
  std::shared_ptr<quad_utils::QuadKD2> quadKD_;

  /// PD gain when foot is in stance
  std::vector<double> stance_kp_;
  std::vector<double> stance_kd_;

  /// PD gain when foot is in swing
  std::vector<double> swing_kp_;
  std::vector<double> swing_kd_;

  /// PD gain when foot is in swing (Cartesian)
  std::vector<double> swing_kp_cart_;
  std::vector<double> swing_kd_cart_;

  /// Last local plan message
  quad_msgs::msg::RobotPlan::SharedPtr last_local_plan_msg_;

  /// Time of last local plan message
  rclcpp::Time last_local_plan_time_;

  /// Bool for whether to override the state machine
  bool override_state_machine_;

  // Absolute Path to ONNX Policy Weights
  std::string model_path_;
};

#endif  // LEG_CONTROLLER_H
