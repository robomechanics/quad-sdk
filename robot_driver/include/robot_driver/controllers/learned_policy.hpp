#ifndef LEARNED_POLICY_H
#define LEARNED_POLICY_H

#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <quad_utils/ros_utils.hpp>
#include <onnxruntime_cxx_api.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <robot_driver/controllers/leg_controller.hpp>

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

//! Implements an abstract class for learned policies.
/*!
   LearnedPolicy provides an abstract learned policy class. It contains
   pure virtual methods for running inference and computing motor commands for each leg to be sent to
   the robot.
*/

class LearnedPolicy : public LegController {
 public:
 /**
  * @brief Constructor for LearnedPolicy
  * @param[in] node Shared pointer to rclcpp::Node
  * @param[in] robot_ns Robot Namespace
  * @return Constructed object of type LearnedPolicy
  */
  LearnedPolicy(rclcpp::Node::SharedPtr node, std::string& robot_ns);

 /**
  * @brief Set the desired proportional and derivative gains for all legs
  * @param[in] stance_kp Stance phase proportional gains
  * @param[in] stance_kd Stance phase derivative gains
  * @param[in] swing_kp Swing phase proportional gains
  * @param[in] swing_kd Swing phase derivative gains
  * @param[in] swing_kp_cart Cartesian Swing phase proportional gains
  * @param[in] swing_kd_cart Cartesian Swing phase derivative gains
  * @param[in] model_path Absolute Path to ONNX Model Weights
  */
  void init(const std::vector<double> &stance_kp,
            const std::vector<double> &stance_kd,
            const std::vector<double> &swing_kp,
            const std::vector<double> &swing_kd,
            const std::vector<double> &swing_kp_cart,
            const std::vector<double> &swing_kd_cart,
            const std::string& model_path);

  void loadONNXModel();
 /**
  * @brief Adjust Positional Targets to Work with Differences between Isaac and Quad-SDK URDF
  * 
  */
  void adjustPositionalTargets();

  void adjustObservationalTargets();

  /**
   * @brief Compute Observations from Robot State
   * @param[in] robot_state_msg Message including robot state
   */
  void computeObservations(const quad_msgs::msg::RobotState &robot_state_msg);

  void runInference();

  /**
   * @brief Update the current velocity command message
   * @param[in] msg Current velocity command vector
   * @param[in] t_now Current time
   */
  void updateCmdVelMsg(Eigen::VectorXd msg, rclcpp::Time &t_now);

  /**
   * @brief Compute the leg command array message for a given current state and
   * reference plan
   * @param[in] robot_state_msg Message including robot state
   * @param[in] leg_command_array_msg Message including leg commands to be sent
   * @param[in] grf_array_msg Message including ground reaction forces
   */
  bool computeLegCommandArray(const quad_msgs::msg::RobotState &robot_state_msg,
                              quad_msgs::msg::LegCommandArray &leg_command_array_msg, 
                              quad_msgs::msg::GRFArray &grf_array_msg);

 protected:

  /// Onnx Runtime Env Object
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "ros2-onnx"};

  /// Onnx Runtime Session Options
  Ort::SessionOptions so_{};

  /// ONNX Tensor Buffer Memory Info
  Ort::MemoryInfo mem_info_{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault)};

  /// Unique Pointer to Onnx Runtime Session
  std::unique_ptr<Ort::Session> session_;

  /// Newest Velocity Command
  Eigen::VectorXd cmd_vel_msg_{Eigen::VectorXd::Zero(3)};

  /// Time of Newest Velocity Command Message
  rclcpp::Time last_cmd_vel_msg_time_;

  /// Action Applied on the Last Inference (Current Joint Positions - Defaults) / Scale Factor
  Eigen::VectorXd prev_action_{Eigen::VectorXd::Zero(12)};

  /// Observation and Action Vectors
  Eigen::VectorXd obs_{Eigen::VectorXd::Zero(48)};
  Eigen::VectorXd actions_{Eigen::VectorXd::Zero(12)};
  Eigen::VectorXd raw_actions_{Eigen::VectorXd::Zero(12)};

  double scale_factor_ = 0.25; // Grabbed Directly From IsaacLab Repo

  Eigen::VectorXd nominal_stance_pose_{Eigen::VectorXd::Zero(12)};

  Eigen::VectorXd temp_actions_{Eigen::VectorXd::Zero(12)};

  bool initialized_ = true;

};
#endif //LEARNED_POLICY_H