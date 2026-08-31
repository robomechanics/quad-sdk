#include "robot_driver/controllers/learned_policy.hpp"

LearnedPolicy::LearnedPolicy(rclcpp::Node::SharedPtr node,
                             const std::string& robot_ns,
                             std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LegController(node, robot_ns, quadKD) {}

std::string LearnedPolicy::resolveModelPath(const std::string& path) const {
  // Absolute paths are honored as-is; anything else is taken relative to the
  // installed robot_driver share directory so the same config works in the
  // devcontainer, on the Jetson, and in a local colcon workspace.
  if (path.empty() || path.front() == '/') return path;
  try {
    return ament_index_cpp::get_package_share_directory("robot_driver") + "/" +
           path;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Could not locate robot_driver share directory (%s); using "
                 "model_path verbatim",
                 e.what());
    return path;
  }
}

void LearnedPolicy::loadPolicyParams() {
  // Every value here mirrors the training env.yaml. Defaults are the stock
  // Isaac-Velocity-Flat-Unitree-Go2-v0 contract; override in the robot config
  // only if the policy was trained with a modified environment.
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_action_scale",
                                  scale_factor_, 0.25);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_lin_vel_scale",
                                  lin_vel_scale_, 1.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_ang_vel_scale",
                                  ang_vel_scale_, 1.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_joint_pos_scale",
                                  joint_pos_scale_, 1.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_joint_vel_scale",
                                  joint_vel_scale_, 1.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_use_lin_vel_obs",
                                  use_lin_vel_obs_, true);
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_kp", policy_kp_,
                                  {25.0, 25.0, 25.0});
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_kd", policy_kd_,
                                  {0.5, 0.5, 0.5});

  std::vector<double> cmd_vel_max;
  quad_utils::loadROSParamDefault(node_, "robot_driver.policy_cmd_vel_max",
                                  cmd_vel_max, {1.0, 1.0, 1.0});
  if (cmd_vel_max.size() == 3) {
    cmd_vel_max_ << cmd_vel_max.at(0), cmd_vel_max.at(1), cmd_vel_max.at(2);
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "policy_cmd_vel_max must have 3 entries (got %zu); keeping "
                "defaults [1.0, 1.0, 1.0]",
                cmd_vel_max.size());
  }

  if (policy_kp_.size() != 3 || policy_kd_.size() != 3) {
    RCLCPP_WARN(node_->get_logger(),
                "policy_kp/policy_kd must have 3 entries; falling back to the "
                "IsaacLab actuator gains (25.0 / 0.5)");
    policy_kp_ = {25.0, 25.0, 25.0};
    policy_kd_ = {0.5, 0.5, 0.5};
  }
}

void LearnedPolicy::init(const std::vector<double>& stance_kp,
                         const std::vector<double>& stance_kd,
                         const std::vector<double>& swing_kp,
                         const std::vector<double>& swing_kd,
                         const std::vector<double>& swing_kp_cart,
                         const std::vector<double>& swing_kd_cart,
                         const std::string& model_path,
                         double policy_inference_rate,
                         const std::vector<double>& stand_joint_angles) {
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
  swing_kp_cart_ = swing_kp_cart;
  swing_kd_cart_ = swing_kd_cart;
  model_path_ = resolveModelPath(model_path);
  policy_inference_rate_ = policy_inference_rate;
  first_inference_ = true;

  loadPolicyParams();
  loadONNXModel();

  // Default joint pose in IsaacLab order (FL,FR,RL,RR grouped by joint type).
  // IsaacLab's Go2 init_state is NOT a single [abd, hip, knee] triple repeated
  // across legs: the hip splays outward (+0.1 left, -0.1 right) and the rear
  // thighs sit 0.2 rad further forward than the front (1.0 vs 0.8). Using a
  // uniform triple here biases every joint_pos observation and every action
  // offset, so it is read as a full 12-vector.
  std::vector<double> default_joint_pos;
  quad_utils::loadROSParamDefault(
      node_, "robot_driver.policy_default_joint_pos", default_joint_pos,
      {0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5});

  if (default_joint_pos.size() == 12) {
    for (int i = 0; i < 12; ++i)
      nominal_stance_pose_(i) = default_joint_pos.at(i);
  } else {
    // Legacy fallback: replicate the [abd, hip, knee] stand pose across legs.
    RCLCPP_WARN(node_->get_logger(),
                "policy_default_joint_pos must have 12 entries (got %zu); "
                "falling back to stand_joint_angles replicated across legs",
                default_joint_pos.size());
    const double abd = stand_joint_angles.at(0);
    const double hip = stand_joint_angles.at(1);
    const double knee = stand_joint_angles.at(2);
    nominal_stance_pose_ << abd, abd, abd, abd, hip, hip, hip, hip, knee, knee,
        knee, knee;
  }

  // Same pose in Quad-SDK order, used as the held command until the first
  // inference succeeds so a failed/late policy never commands all-zero angles.
  nominal_stance_pose_sdk_ << nominal_stance_pose_(0), nominal_stance_pose_(4),
      nominal_stance_pose_(8), nominal_stance_pose_(2), nominal_stance_pose_(6),
      nominal_stance_pose_(10), nominal_stance_pose_(1),
      nominal_stance_pose_(5), nominal_stance_pose_(9), nominal_stance_pose_(3),
      nominal_stance_pose_(7), nominal_stance_pose_(11);
  actions_ = nominal_stance_pose_sdk_;

  // The policy contract defines obs 36-47 as the previous raw ONNX output,
  // zero-initialized on startup and after any reset.
  prev_action_.setZero();
  raw_actions_.setZero();

  last_cmd_vel_msg_time_ = node_->now();
  last_inference_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
              "Loaded Learned Policy at %s (%.1f Hz, action scale %.3f, kp "
              "%.1f, kd %.2f)",
              model_path_.c_str(), policy_inference_rate_, scale_factor_,
              policy_kp_.at(0), policy_kd_.at(0));
}

void LearnedPolicy::loadONNXModel() {
  /// Try loading and Initalizing an Onnx Runtime Session
  try {
    so_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // Specify thread counts explicitly. Without this, onnxruntime tries to pin
    // its intra-op threads to specific cores via pthread_setaffinity_np, which
    // fails on Jetson/Tegra (EINVAL) and floods the log. Setting the counts
    // explicitly disables the affinity pinning.
    so_.SetIntraOpNumThreads(1);
    so_.SetInterOpNumThreads(1);

    // Enable CUDA execution provider for GPU inference.
    OrtCUDAProviderOptions cuda_options{};
    cuda_options.device_id = 0;
    so_.AppendExecutionProvider_CUDA(cuda_options);

    if (!std::filesystem::exists(model_path_)) {
      RCLCPP_ERROR(node_->get_logger(), "ONNX file not found: %s",
                   model_path_.c_str());
      return;
    }

    session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), so_);
    RCLCPP_INFO(node_->get_logger(), "Session created for %s",
                model_path_.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create ONNX Session: %s",
                 e.what());
    session_.reset();
    return;
  }
  /// Verify the model matches the observation/action contract this controller
  /// builds. A silent mismatch here is the most common deployment bug, so it
  /// is a hard failure rather than a warning.
  try {
    auto in_shape =
        session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    auto out_shape =
        session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

    const int64_t in_dim = in_shape.empty() ? -1 : in_shape.back();
    const int64_t out_dim = out_shape.empty() ? -1 : out_shape.back();
    RCLCPP_INFO(node_->get_logger(), "ONNX I/O: input [.., %lld] -> output [.., %lld]",
                static_cast<long long>(in_dim), static_cast<long long>(out_dim));

    if (in_dim != kObsDim || out_dim != 3 * num_feet_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "ONNX shape mismatch: expected input %d and output %d, got "
                   "%lld and %lld. Refusing to run this policy.",
                   kObsDim, 3 * num_feet_, static_cast<long long>(in_dim),
                   static_cast<long long>(out_dim));
      session_.reset();
    }
  } catch (const Ort::Exception& e) {
    RCLCPP_WARN(node_->get_logger(), "I/O introspection failed: %s", e.what());
    // Session is still valid; you can proceed to Run() if you know your I/O
    // contract.
  } catch (const std::exception& e) {
    RCLCPP_WARN(node_->get_logger(), "I/O introspection failed (std): %s",
                e.what());
  }
}

void LearnedPolicy::computeObservations(
    const quad_msgs::msg::RobotState& robot_state_msg) {
  // Builds the 48-value IsaacLab velocity-task observation, in this order:
  //   0-2   base linear velocity, body frame        (m/s)
  //   3-5   base angular velocity, body frame       (rad/s)
  //   6-8   projected gravity, body frame           (unit)
  //   9-11  command [vx, vy, yaw_rate], body frame
  //   12-23 joint positions relative to default     (rad)
  //   24-35 joint velocities                        (rad/s)
  //   36-47 previous raw policy action              (unit)
  // All blocks are in IsaacLab joint order and, for the stock task, unscaled.

  Eigen::VectorXd joint_positions(3 * num_feet_),
      joint_velocities(3 * num_feet_), raw_joint_positions(3 * num_feet_),
      raw_joint_velocities(3 * num_feet_), body_state(12);

  quad_utils::vectorToEigen(robot_state_msg.joints.position,
                            raw_joint_positions);
  quad_utils::vectorToEigen(robot_state_msg.joints.velocity,
                            raw_joint_velocities);
  body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg.body);

  const Eigen::Quaterniond quat(robot_state_msg.body.pose.orientation.w,
                                robot_state_msg.body.pose.orientation.x,
                                robot_state_msg.body.pose.orientation.y,
                                robot_state_msg.body.pose.orientation.z);

  // body.twist.linear is the world-frame estimate, so rotate it into the base
  // frame the policy was trained on. On hardware without a trustworthy linear
  // velocity estimate, set policy_use_lin_vel_obs false to feed zeros instead.
  Eigen::Vector3d base_lin_vel = Eigen::Vector3d::Zero();
  if (use_lin_vel_obs_) {
    const Eigen::Vector3d lin_vel_world(body_state(6), body_state(7),
                                        body_state(8));
    base_lin_vel = quat.conjugate() * lin_vel_world;
  }

  // body.twist.angular is the raw IMU gyro in both the hardware path and the
  // EKF, so it is already the body-frame rate IsaacLab expects.
  Eigen::Vector3d base_ang_vel(body_state(9), body_state(10), body_state(11));

  // Projected gravity: world gravity rotated into the base frame. Level robot
  // gives approximately [0, 0, -1] under the IsaacLab convention.
  const Eigen::Vector3d g_world(0.0, 0.0, -1.0);
  const Eigen::Vector3d proj_gravity = quat.conjugate() * g_world;

  // Clip the commanded velocity to the ranges the policy was trained over.
  Eigen::Vector3d vel_cmd(cmd_vel_msg_(0), cmd_vel_msg_(1), cmd_vel_msg_(5));
  vel_cmd = vel_cmd.cwiseMin(cmd_vel_max_).cwiseMax(-cmd_vel_max_);

  // Reorder joints from Quad-SDK to IsaacLab order.
  //   Quad-SDK: FL(abd 0, hip 1, knee 2), RL(3,4,5), FR(6,7,8), RR(9,10,11)
  //   IsaacLab: hips  (FL 0, FR 1, RL 2, RR 3)
  //             thighs(FL 4, FR 5, RL 6, RR 7)
  //             calves(FL 8, FR 9, RL 10, RR 11)
  joint_positions << raw_joint_positions(0), raw_joint_positions(6),
      raw_joint_positions(3), raw_joint_positions(9), raw_joint_positions(1),
      raw_joint_positions(7), raw_joint_positions(4), raw_joint_positions(10),
      raw_joint_positions(2), raw_joint_positions(8), raw_joint_positions(5),
      raw_joint_positions(11);

  joint_velocities << raw_joint_velocities(0), raw_joint_velocities(6),
      raw_joint_velocities(3), raw_joint_velocities(9), raw_joint_velocities(1),
      raw_joint_velocities(7), raw_joint_velocities(4),
      raw_joint_velocities(10), raw_joint_velocities(2),
      raw_joint_velocities(8), raw_joint_velocities(5),
      raw_joint_velocities(11);

  // joint_pos_rel is measured minus the default pose; joint_vel is absolute
  // (the Go2 default joint velocity is zero, so joint_vel_rel == joint_vel).
  joint_positions -= nominal_stance_pose_;

  // Observation scales. Unity for the stock task — IsaacLab leaves every
  // ObsTerm.scale as None — but kept configurable for retrained policies.
  base_lin_vel *= lin_vel_scale_;
  base_ang_vel *= ang_vel_scale_;
  joint_positions *= joint_pos_scale_;
  joint_velocities *= joint_vel_scale_;

  // obs 36-47 is the previous raw ONNX output, held at zero until the first
  // inference completes.
  prev_action_ = raw_actions_;

  obs_.resize(kObsDim);
  obs_ << base_lin_vel, base_ang_vel, proj_gravity, vel_cmd, joint_positions,
      joint_velocities, prev_action_;
}

void LearnedPolicy::runInference() {
  // Run Inference and Update Previous Action
  if (!session_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "ONNX Session Not Ready");
    return;
  }

  Eigen::VectorXf obs_f = obs_.cast<float>();
  const int64_t input_shape[2] = {1, static_cast<int64_t>(obs_f.size())};

  // Create an ONNX Runtime Tensor that points to CPU Buffer
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info_, obs_f.data(), obs_f.size(), input_shape, 2);

  Ort::AllocatorWithDefaultOptions alloc;
  auto in_name_alloc = session_->GetInputNameAllocated(0, alloc);
  auto out_name_alloc = session_->GetOutputNameAllocated(0, alloc);
  const char* in_names[] = {in_name_alloc.get()};
  const char* out_names[] = {out_name_alloc.get()};

  auto outs = session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor,
                            1, out_names, 1);

  if (outs.size() != 1 || !outs[0].IsTensor()) {
    RCLCPP_ERROR(node_->get_logger(), "Unexpected ONNX output");
    return;
  }

  auto out_info = outs[0].GetTensorTypeAndShapeInfo();
  auto out_shape = out_info.GetShape();

  int64_t out_elems = 1;
  for (auto d : out_shape) out_elems *= (d < 0 ? 1 : d);

  if (out_elems != 3 * num_feet_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ONNX returned %lld actions, expected %d; holding last "
                 "command",
                 static_cast<long long>(out_elems), 3 * num_feet_);
    return;
  }

  float* y = outs[0].GetTensorMutableData<float>();
  Eigen::VectorXd new_actions(out_elems);
  for (int64_t i = 0; i < out_elems; ++i)
    new_actions(i) = static_cast<double>(y[i]);

  // A NaN/inf here would propagate straight into a position setpoint, so drop
  // the whole inference and hold the last valid command instead.
  if (!new_actions.allFinite()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ONNX produced non-finite actions; holding last command");
    return;
  }
  raw_actions_ = new_actions;

  // q_target = q_default + action_scale * raw_action, in IsaacLab order.
  const Eigen::VectorXd unordered_actions =
      raw_actions_ * scale_factor_ + nominal_stance_pose_;

  // Reorder back into Quad-SDK order (inverse of the observation mapping):
  //   Quad-SDK FL(abd 0, hip 1, knee 2), RL(3,4,5), FR(6,7,8), RR(9,10,11)
  actions_ << unordered_actions(0), unordered_actions(4), unordered_actions(8),
      unordered_actions(2), unordered_actions(6), unordered_actions(10),
      unordered_actions(1), unordered_actions(5), unordered_actions(9),
      unordered_actions(3), unordered_actions(7), unordered_actions(11);
}

bool LearnedPolicy::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& /*grf_array_msg*/) {
  // Safety: return false if cmd_vel is stale
  if ((node_->now() - last_cmd_vel_msg_time_).seconds() >= 0.1) {
    return false;
  }

  // Run inference at policy_inference_rate_ (50 Hz for this policy: the
  // training env used dt 0.005 with decimation 4), not every tick.
  auto now = node_->now();
  if (first_inference_ ||
      (now - last_inference_time_).seconds() >= 1.0 / policy_inference_rate_) {
    computeObservations(robot_state_msg);
    runInference();
    last_inference_time_ = now;
    first_inference_ = false;
  }

  // PD tracking at full loop rate (500 Hz) using cached position targets
  leg_command_array_msg.leg_commands.resize(num_feet_);
  for (int i = 0; i < num_feet_; ++i) {
    auto& leg = leg_command_array_msg.leg_commands.at(i);
    leg.motor_commands.resize(3);

    for (int j = 0; j < 3; ++j) {
      const int idx = 3 * i + j;
      auto& cmd = leg.motor_commands.at(j);

      // Position targets from last inference, tracked by PD in robot_driver
      cmd.pos_setpoint = actions_(idx);
      cmd.vel_setpoint = 0.0;
      cmd.torque_ff = 0.0;

      // Gains matched to the IsaacLab actuator the policy was trained against
      // (DCMotor stiffness 25.0 / damping 0.5), not the model-based stance
      // gains, which are much stiffer.
      cmd.kp = policy_kp_.at(j);
      cmd.kd = policy_kd_.at(j);
    }
  }
  return true;
}

void LearnedPolicy::updateCmdVelMsg(Eigen::VectorXd msg, rclcpp::Time& t_now) {
  cmd_vel_msg_ = msg;
  last_cmd_vel_msg_time_ = t_now;
}

void LearnedPolicy::updateImuMsg(const sensor_msgs::msg::Imu& imu_msg) {
  last_imu_msg_ = imu_msg;
}
