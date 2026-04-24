#ifndef LEARNED_POLICY_H
#define LEARNED_POLICY_H

#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <quad_msgs/msg/leg_command_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
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
   pure virtual methods for running inference and computing motor commands for
   each leg to be sent to the robot.
*/

class LearnedPolicy : public LegController {
 public:
  /**
   * @brief Constructor for LearnedPolicy
   * @return Constructed object of type LearnedPolicy
   */
  LearnedPolicy(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Set the desired proportional and derivative gains for all legs
   * @param[in] kp Proportional gains
   * @param[in] kd Derivative Gains
   */
  void init(const std::vector<double>& stance_kp,
            const std::vector<double>& stance_kd,
            const std::vector<double>& swing_kp,
            const std::vector<double>& swing_kd,
            const std::vector<double>& swing_kp_cart,
            const std::vector<double>& swing_kd_cart,
            const std::string& model_path,
            double policy_inference_rate = 50.0,
            const std::vector<double>& stand_joint_angles = {0.0, 0.8, -1.5});

  void loadONNXModel();
  /**
   * @brief Adjust Positional Targets to Work with Differences between Isaac and
   * Quad-SDK URDF
   *
   */
  void adjustPositionalTargets();

  void adjustObservationalTargets();

  void computeObservations(const quad_msgs::msg::RobotState& robot_state_msg);

  void runInference();

  void updateCmdVelMsg(Eigen::VectorXd msg, rclcpp::Time& t_now);

  void updateImuMsg(const sensor_msgs::msg::Imu& imu_msg);

  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg);

 protected:
  /// Onnx Runtime Env Object
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "ros2-onnx"};

  /// Onnx Runtime Session Options
  Ort::SessionOptions so_{};

  /// ONNX Tensor Buffer Memory Info
  Ort::MemoryInfo mem_info_{
      Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault)};

  /// Unique Pointer to Onnx Runtime Session
  std::unique_ptr<Ort::Session> session_;

  /// Cached IMU message (for acceleration access)
  sensor_msgs::msg::Imu last_imu_msg_;

  /// Newest Velocity Command
  Eigen::VectorXd cmd_vel_msg_{Eigen::VectorXd::Zero(3)};

  /// Time of Newest Velocity Command Message
  rclcpp::Time last_cmd_vel_msg_time_;

  /// Action Applied on the Last Inference (Current Joint Positions - Defaults)
  /// / Scale Factor
  Eigen::VectorXd prev_action_{Eigen::VectorXd::Zero(12)};

  /// Observation and Action Vectors
  Eigen::VectorXd obs_{Eigen::VectorXd::Zero(48)};
  Eigen::VectorXd actions_{Eigen::VectorXd::Zero(12)};
  Eigen::VectorXd raw_actions_{Eigen::VectorXd::Zero(12)};

  /// Policy inference rate (Hz), PD tracking at loop rate
  double policy_inference_rate_ = 50.0;

  /// Timestamp of last inference run
  rclcpp::Time last_inference_time_;

  /// Whether first inference has been run yet
  bool first_inference_ = true;

  double scale_factor_ = 0.25;  // Grabbed Directly From IsaacLab Repo

  Eigen::VectorXd nominal_stance_pose_{Eigen::VectorXd::Zero(12)};

  Eigen::VectorXd temp_actions_{Eigen::VectorXd::Zero(12)};

  bool initialized_ = true;
};
#endif  // LEARNED_POLICY_H
