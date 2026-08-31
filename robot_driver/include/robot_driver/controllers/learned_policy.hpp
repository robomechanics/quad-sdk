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
  /// Observation width of the IsaacLab velocity-flat contract:
  /// base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3) +
  /// velocity_commands(3) + joint_pos_rel(12) + joint_vel(12) + actions(12)
  static constexpr int kObsDim = 48;

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
   * @brief Load the IsaacLab observation/action contract from the parameter
   * server (scales, default joint pose, command clipping, PD gains).
   *
   * These must mirror the training env.yaml exactly. For the stock
   * Isaac-Velocity-Flat-Unitree-Go2-v0 task every observation scale is unity
   * (IsaacLab leaves ObsTerm.scale as None) and the action term is
   * scale=0.25 with use_default_offset=true.
   */
  void loadPolicyParams();

  /**
   * @brief Resolve a possibly-relative ONNX path against the robot_driver
   * package share directory so configs are not tied to a container layout.
   * @param[in] path Absolute path, or path relative to share/robot_driver
   * @return Absolute path to the ONNX file
   */
  std::string resolveModelPath(const std::string& path) const;

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

  /// Raw ONNX output of the previous inference (obs values 36-47). Zero before
  /// the first inference and after any reset, per the policy contract.
  Eigen::VectorXd prev_action_{Eigen::VectorXd::Zero(12)};

  /// Observation and Action Vectors
  Eigen::VectorXd obs_{Eigen::VectorXd::Zero(kObsDim)};
  Eigen::VectorXd actions_{Eigen::VectorXd::Zero(12)};
  Eigen::VectorXd raw_actions_{Eigen::VectorXd::Zero(12)};

  /// Policy inference rate (Hz), PD tracking at loop rate
  double policy_inference_rate_ = 50.0;

  /// Timestamp of last inference run
  rclcpp::Time last_inference_time_;

  /// Whether first inference has been run yet
  bool first_inference_ = true;

  /// IsaacLab JointPositionAction scale (env.yaml actions.joint_pos.scale)
  double scale_factor_ = 0.25;

  /// IsaacLab observation-term scales. The stock velocity task leaves every
  /// ObsTerm.scale as None, i.e. unity — override only if the training config
  /// actually sets them.
  double lin_vel_scale_ = 1.0;
  double ang_vel_scale_ = 1.0;
  double joint_pos_scale_ = 1.0;
  double joint_vel_scale_ = 1.0;

  /// Whether to feed the estimated base linear velocity (obs 0-2). Set false
  /// on hardware with no reliable linear-velocity estimate; the policy then
  /// sees zeros there rather than a garbage estimate.
  bool use_lin_vel_obs_ = true;

  /// Command clipping, [vx, vy, yaw_rate], matched to the training ranges
  Eigen::Vector3d cmd_vel_max_{1.0, 1.0, 1.0};

  /// Default joint pose in IsaacLab order (obs 12-23 reference and the offset
  /// added to the scaled action)
  Eigen::VectorXd nominal_stance_pose_{Eigen::VectorXd::Zero(12)};

  /// Same pose reordered into Quad-SDK order, used as the command held before
  /// the first successful inference
  Eigen::VectorXd nominal_stance_pose_sdk_{Eigen::VectorXd::Zero(12)};

  /// PD gains applied to the policy's position targets. Separate from the
  /// LegController stance gains so the learned controller can run the
  /// IsaacLab actuator gains without retuning the model-based controllers.
  std::vector<double> policy_kp_{25.0, 25.0, 25.0};
  std::vector<double> policy_kd_{0.5, 0.5, 0.5};
};
#endif  // LEARNED_POLICY_H
