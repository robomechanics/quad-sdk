#include "robot_driver/controllers/forl_policy.hpp"

ForlPolicy::ForlPolicy(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                       std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LearnedPolicy(node, robot_ns, quadKD) {}

void ForlPolicy::init(const std::vector<double>& stance_kp,
                      const std::vector<double>& stance_kd,
                      const std::vector<double>& swing_kp,
                      const std::vector<double>& swing_kd,
                      const std::vector<double>& swing_kp_cart,
                      const std::vector<double>& swing_kd_cart,
                      const std::string& model_path,
                      double policy_inference_rate,
                      const std::vector<double>& stand_joint_angles) {
  // Loads the ONNX session and stores the inference rate. The yaml gains and
  // stand pose it also stores are not used by this controller.
  LearnedPolicy::init(stance_kp, stance_kd, swing_kp, swing_kd, swing_kp_cart,
                      swing_kd_cart, model_path, policy_inference_rate,
                      stand_joint_angles);

  std::vector<double> default_angles, cmd_limits;
  quad_utils::loadROSParamDefault(node_, "robot_driver.forl_kp", kp_, 25.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.forl_kd", kd_, 0.5);
  quad_utils::loadROSParamDefault(node_, "robot_driver.forl_action_scale",
                                  action_scale_, 0.5);
  quad_utils::loadROSParamDefault(node_,
                                  "robot_driver.forl_default_joint_angles",
                                  default_angles,
                                  std::vector<double>{0.0, 0.9, -1.8});
  quad_utils::loadROSParamDefault(node_, "robot_driver.forl_cmd_limits",
                                  cmd_limits,
                                  std::vector<double>{1.0, 0.4, 1.0});
  if (default_angles.size() != 3 || cmd_limits.size() != 3 || kp_ < 0.0 ||
      kd_ < 0.0 || action_scale_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ForlPolicy: invalid robot_driver.forl_* parameters");
    model_ok_ = false;
    return;
  }
  default_pose_ = forl::defaultPose(
      Eigen::Vector3d(default_angles[0], default_angles[1], default_angles[2]));
  cmd_limits_ << std::abs(cmd_limits[0]), std::abs(cmd_limits[1]),
      std::abs(cmd_limits[2]);

  last_action_.setZero();
  have_targets_ = false;
  model_ok_ = modelShapeIsValid();

  RCLCPP_INFO(node_->get_logger(),
              "ForlPolicy: kp=%.1f kd=%.2f action_scale=%.2f default pose "
              "[%.2f, %.2f, %.2f] command limits [%.2f, %.2f, %.2f]%s",
              kp_, kd_, action_scale_, default_angles[0], default_angles[1],
              default_angles[2], cmd_limits_(0), cmd_limits_(1), cmd_limits_(2),
              model_ok_ ? "" : " (MODEL NOT USABLE)");
}

bool ForlPolicy::modelShapeIsValid() {
  if (!session_) {
    RCLCPP_ERROR(node_->get_logger(), "ForlPolicy: no ONNX session");
    return false;
  }
  try {
    // Keep the TypeInfo objects alive while their shape views are read.
    Ort::TypeInfo in_info = session_->GetInputTypeInfo(0);
    Ort::TypeInfo out_info = session_->GetOutputTypeInfo(0);
    const auto in_shape = in_info.GetTensorTypeAndShapeInfo().GetShape();
    const auto out_shape = out_info.GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.empty() || out_shape.empty() ||
        in_shape.back() != forl::kObsDim ||
        out_shape.back() != forl::kNumJoints) {
      RCLCPP_ERROR(node_->get_logger(),
                   "ForlPolicy: model must map %d observations to %d actions",
                   forl::kObsDim, forl::kNumJoints);
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "ForlPolicy: cannot read model shape: %s",
                 e.what());
    return false;
  }
  return true;
}

bool ForlPolicy::stepPolicy(const quad_msgs::msg::RobotState& robot_state_msg) {
  const auto& joints = robot_state_msg.joints;
  if (joints.position.size() < forl::kNumJoints ||
      joints.velocity.size() < forl::kNumJoints) {
    return false;
  }
  forl::Vector12d q, qd;
  for (int i = 0; i < forl::kNumJoints; ++i) {
    q(i) = joints.position[i];
    qd(i) = joints.velocity[i];
  }
  const auto& orientation = robot_state_msg.body.pose.orientation;
  const Eigen::Quaterniond q_world_body(orientation.w, orientation.x,
                                        orientation.y, orientation.z);
  if (q_world_body.norm() < 1e-6) {
    return false;  // no orientation estimate yet
  }
  const auto& w = robot_state_msg.body.twist.angular;
  // cmd_vel_msg_ is a twist [vx, vy, vz, wx, wy, wz].
  const Eigen::Vector3d command = forl::clipCommand(
      Eigen::Vector3d(cmd_vel_msg_(0), cmd_vel_msg_(1), cmd_vel_msg_(5)),
      cmd_limits_);

  const forl::ObsVector obs = forl::assembleObservation(
      Eigen::Vector3d(w.x, w.y, w.z), forl::projectedGravity(q_world_body), q,
      qd, last_action_, command, default_pose_);
  if (!obs.allFinite()) {
    return false;
  }

  obs_ = obs.cast<double>();
  raw_actions_.setConstant(forl::kNumJoints,
                           std::numeric_limits<double>::quiet_NaN());
  LearnedPolicy::runInference();
  if (raw_actions_.size() != forl::kNumJoints || !raw_actions_.allFinite()) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "ForlPolicy: inference failed or returned non-finite "
                          "actions");
    return false;
  }

  last_action_ = raw_actions_.cwiseMin(1.0).cwiseMax(-1.0);
  targets_ = forl::actionToTargets(last_action_, default_pose_, action_scale_);
  have_targets_ = true;
  return true;
}

bool ForlPolicy::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& /*grf_array_msg*/) {
  // Returning false makes the driver hold its stand pose.
  if (!model_ok_) {
    return false;
  }
  const auto now = node_->now();
  if ((now - last_cmd_vel_msg_time_).seconds() >= 0.1) {
    // Stale command: forget the action history so the policy restarts clean.
    last_action_.setZero();
    have_targets_ = false;
    first_inference_ = true;
    return false;
  }

  if (first_inference_ ||
      (now - last_inference_time_).seconds() >= 1.0 / policy_inference_rate_) {
    if (!stepPolicy(robot_state_msg)) {
      return false;
    }
    last_inference_time_ = now;
    first_inference_ = false;
  }
  if (!have_targets_) {
    return false;
  }

  leg_command_array_msg.leg_commands.resize(num_feet_);
  for (int i = 0; i < num_feet_; ++i) {
    auto& leg = leg_command_array_msg.leg_commands.at(i);
    leg.motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {
      auto& cmd = leg.motor_commands.at(j);
      cmd.pos_setpoint = targets_(3 * i + j);
      cmd.vel_setpoint = 0.0;
      cmd.torque_ff = 0.0;
      cmd.kp = kp_;
      cmd.kd = kd_;
    }
  }
  return true;
}
