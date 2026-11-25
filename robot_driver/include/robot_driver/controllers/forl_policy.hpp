#ifndef FORL_POLICY_H
#define FORL_POLICY_H

#include <string>
#include <vector>

#include "robot_driver/controllers/forl_policy_math.hpp"
#include "robot_driver/controllers/learned_policy.hpp"

//! Runs a FORL (forlax) Go2 joystick policy exported to ONNX.
/*!
   ForlPolicy reuses LearnedPolicy's ONNX session, command and IMU plumbing and
   replaces the observation and action conventions with those of the MuJoCo
   MJX Go2 joystick task the policy was trained on (see forl_policy_math.hpp).
   The PD gains, action scale, default pose and command limits are those of
   training and are read from robot_driver.forl_* parameters, so the gains in
   the robot's yaml, which are tuned for the model-based controllers, are not
   used.
*/
class ForlPolicy : public LearnedPolicy {
 public:
  ForlPolicy(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
             std::shared_ptr<quad_utils::QuadKD2> quadKD);

  void init(const std::vector<double>& stance_kp,
            const std::vector<double>& stance_kd,
            const std::vector<double>& swing_kp,
            const std::vector<double>& swing_kd,
            const std::vector<double>& swing_kp_cart,
            const std::vector<double>& swing_kd_cart,
            const std::string& model_path, double policy_inference_rate,
            const std::vector<double>& stand_joint_angles) override;

  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg) override;

 private:
  /// Check that the loaded model maps [1, 45] to [1, 12].
  bool modelShapeIsValid();

  /// Build the observation, run the network and update the joint targets.
  bool stepPolicy(const quad_msgs::msg::RobotState& robot_state_msg);

  /// Training PD gains, applied to every joint.
  double kp_ = 25.0;
  double kd_ = 0.5;

  /// Joint target = default pose + action_scale * action.
  double action_scale_ = 0.5;

  /// Default joint pose in policy order.
  forl::Vector12d default_pose_ =
      forl::defaultPose(Eigen::Vector3d(0.0, 0.9, -1.8));

  /// Symmetric command limits (vx, vy, yaw rate) of training.
  Eigen::Vector3d cmd_limits_{1.0, 0.4, 1.0};

  /// Previous raw action in policy order; zero before the first inference.
  forl::Vector12d last_action_ = forl::Vector12d::Zero();

  /// Joint position targets in Quad-SDK order.
  forl::Vector12d targets_ = forl::Vector12d::Zero();

  /// False until a model with the expected shapes is loaded.
  bool model_ok_ = false;

  /// False until the first successful inference.
  bool have_targets_ = false;
};

#endif  // FORL_POLICY_H
