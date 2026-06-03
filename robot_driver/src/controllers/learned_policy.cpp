#include "robot_driver/controllers/learned_policy.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;
constexpr double kCmdVelTimeoutSec = 0.5;

// Joint-frame conversions between the quad-sdk go2w URDF and the IsaacLab
// (robot_lab) training URDF, derived by matching link orientations:
//   thigh: training rpy(0,0,0) axis(0,1,0) vs quad-sdk rpy(0,pi/2,0)
//          axis(0,-1,0)  =>  q_isaac = pi/2 - q_quad  (sign flip + offset),
//          so thigh velocity also flips sign.
//   calf : training rpy(0,0,0) vs quad-sdk rpy(0,-pi,0), same axis(0,1,0)
//          =>  q_isaac = q_quad - pi  (no velocity sign change).
//   abad and wheel frames are identical in both URDFs (identity).
// "Hip" in these names refers to the thigh joint (quad-sdk leg ordering is
// abad, thigh, knee).
double quadGo2wHipToRobotLab(double q) { return kHalfPi - q; }
double quadGo2wHipVelToRobotLab(double dq) { return -dq; }
double quadGo2wKneeToRobotLab(double q) { return q - kPi; }
double quadGo2wKneeVelToRobotLab(double dq) { return dq; }
double robotLabHipToQuadGo2w(double q) { return kHalfPi - q; }
double robotLabKneeToQuadGo2w(double q) { return q + kPi; }
}  // namespace

LearnedPolicy::LearnedPolicy(rclcpp::Node::SharedPtr node,
                             const std::string& robot_ns,
                             std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LegController(node, robot_ns, quadKD) {}

void LearnedPolicy::init(const std::vector<double>& stance_kp,
                         const std::vector<double>& stance_kd,
                         const std::vector<double>& swing_kp,
                         const std::vector<double>& swing_kd,
                         const std::vector<double>& swing_kp_cart,
                         const std::vector<double>& swing_kd_cart,
                         const std::string& model_path,
                         double policy_inference_rate,
                         const std::vector<double>& stand_joint_angles) {
  // Initalize the Path to the Model Onnx File
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
  swing_kp_cart_ = swing_kp_cart;
  swing_kd_cart_ = swing_kd_cart;
  model_path_ = model_path;
  is_go2w_policy_ = (model_path_.find("go2w") != std::string::npos);
  policy_inference_rate_ = policy_inference_rate;
  first_inference_ = true;
  warned_extra_actions_ = false;
  initialized_ = true;
  prev_action_.setZero(is_go2w_policy_ ? 16 : 12);
  raw_actions_.setZero(is_go2w_policy_ ? 16 : 12);
  wheel_actions_.setZero();
  loadONNXModel();
  // Build nominal stance in Isaac ordering (FL,FR,RL,RR grouped by joint type)
  // stand_joint_angles is [abd, hip, knee] from robot_driver.yaml
  double abd = stand_joint_angles.at(0);
  double hip = stand_joint_angles.at(1);
  double knee = stand_joint_angles.at(2);
  nominal_stance_pose_ << abd, abd, abd, abd, hip, hip, hip, hip, knee, knee,
      knee, knee;
  last_cmd_vel_msg_time_ = node_->now();
  last_inference_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(), "Loaded %s Learned Policy at %s (%.1f Hz)",
              is_go2w_policy_ ? "Go2-W" : "Go2", model_path_.c_str(),
              policy_inference_rate_);
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
  /// Debugging Step to Visualize the Network Input and Output Shapes
  try {
    auto in_shape =
        session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    auto out_shape =
        session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() >= 2) {
      RCLCPP_INFO(node_->get_logger(), "input: shape [%lld,%lld]",
                  (long long)in_shape[0], (long long)in_shape[1]);
    } else {
      RCLCPP_INFO(node_->get_logger(), "input: shape [?]");
    }
    if (out_shape.size() >= 2) {
      RCLCPP_INFO(node_->get_logger(), "output: shape [%lld,%lld]",
                  (long long)out_shape[0], (long long)out_shape[1]);
    } else {
      RCLCPP_INFO(node_->get_logger(), "output: shape [?]");
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
  // May Require Changes if your Policy has Different Inputs

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3 * num_feet_),
      joint_velocities(3 * num_feet_), raw_joint_positions(3 * num_feet_),
      raw_joint_velocities(3 * num_feet_), body_state(12);

  quad_utils::vectorToEigen(robot_state_msg.joints.position,
                            raw_joint_positions);
  quad_utils::vectorToEigen(robot_state_msg.joints.velocity,
                            raw_joint_velocities);
  body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg.body);

  Eigen::Vector3d base_lin_vel, base_ang_vel, base_lin_accel, base_orientation,
      vel_cmd;
  base_lin_vel << body_state(6), body_state(7), body_state(8);
  base_ang_vel << body_state(9), body_state(10), body_state(11);
  base_lin_accel << last_imu_msg_.linear_acceleration.x,
      last_imu_msg_.linear_acceleration.y, last_imu_msg_.linear_acceleration.z;
  base_orientation << body_state(3), body_state(4), body_state(5);
  vel_cmd << cmd_vel_msg_(0), cmd_vel_msg_(1), cmd_vel_msg_(5);

  // Clip the Commanded Velocity within Trained Bounds. Training command ranges
  // (velocity_env_cfg.py base_velocity): lin_vel_x/y and ang_vel_z all (-1, 1).
  const Eigen::Vector3d vmin(-1.0, -1.0, -1.0);
  const Eigen::Vector3d vmax(1.0, 1.0, 1.0);
  vel_cmd = vel_cmd.cwiseMin(vmax).cwiseMax(vmin);

  // Compute the Projected Gravity in the Body Frame
  const Eigen::Matrix3d R_bw =
      (Eigen::AngleAxisd(body_state(5), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(body_state(4), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(body_state(3), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const Eigen::Vector3d g_world(
      0.0, 0.0, -1.0);  // Change to  (0,0,-1.0) for Unit Proj Gravity
  const Eigen::Vector3d proj_gravity1 = R_bw.transpose() * g_world;

  Eigen::Quaterniond quat(robot_state_msg.body.pose.orientation.w,
                          robot_state_msg.body.pose.orientation.x,
                          robot_state_msg.body.pose.orientation.y,
                          robot_state_msg.body.pose.orientation.z);
  const Eigen::Vector3d proj_gravity = quat.conjugate() * g_world;

  // Reorder Raw Joint Positions and Raw Joint Velocities in Quad-SDK Notation
  // to match Isaac Notation, See Below: QuadSDK Order FL(hip 0, thigh 1, knee
  // 2), RL (hip 3, thigh 4, knee 5), FR(hip 6, thigh 7, knee 8), RR(hip 9,
  // thigh 10, knee 11) IsaacLab Order FL, FR, RL, RR (hips, thighs, knees)

  if (is_go2w_policy_) {
    // RobotLab Go2-W order is leg-major: FR, FL, RR, RL. The policy observes
    // 12 leg positions, 16 joint velocities (legs + wheels), and 16 previous
    // actions (12 leg position actions + 4 wheel velocity actions).
    Eigen::VectorXd leg_joint_positions(12), joint_velocities_with_wheels(16);
    leg_joint_positions << raw_joint_positions(6),
        quadGo2wHipToRobotLab(raw_joint_positions(7)),
        quadGo2wKneeToRobotLab(raw_joint_positions(8)), raw_joint_positions(0),
        quadGo2wHipToRobotLab(raw_joint_positions(1)),
        quadGo2wKneeToRobotLab(raw_joint_positions(2)), raw_joint_positions(9),
        quadGo2wHipToRobotLab(raw_joint_positions(10)),
        quadGo2wKneeToRobotLab(raw_joint_positions(11)), raw_joint_positions(3),
        quadGo2wHipToRobotLab(raw_joint_positions(4)),
        quadGo2wKneeToRobotLab(raw_joint_positions(5));

    joint_velocities_with_wheels << raw_joint_velocities(6),
        quadGo2wHipVelToRobotLab(raw_joint_velocities(7)),
        quadGo2wKneeVelToRobotLab(raw_joint_velocities(8)),
        raw_joint_velocities(0),
        quadGo2wHipVelToRobotLab(raw_joint_velocities(1)),
        quadGo2wKneeVelToRobotLab(raw_joint_velocities(2)),
        raw_joint_velocities(9),
        quadGo2wHipVelToRobotLab(raw_joint_velocities(10)),
        quadGo2wKneeVelToRobotLab(raw_joint_velocities(11)),
        raw_joint_velocities(3),
        quadGo2wHipVelToRobotLab(raw_joint_velocities(4)),
        quadGo2wKneeVelToRobotLab(raw_joint_velocities(5)),
        raw_joint_velocities.size() > 14 ? raw_joint_velocities(14) : 0.0,
        raw_joint_velocities.size() > 12 ? raw_joint_velocities(12) : 0.0,
        raw_joint_velocities.size() > 15 ? raw_joint_velocities(15) : 0.0,
        raw_joint_velocities.size() > 13 ? raw_joint_velocities(13) : 0.0;

    Eigen::VectorXd robotlab_nominal_stance(12);
    robotlab_nominal_stance << 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8,
        -1.5, 0.0, 0.8, -1.5;

    leg_joint_positions -= robotlab_nominal_stance;
    base_ang_vel *= 0.25;
    joint_velocities_with_wheels *= 0.05;

    // Training uses mdp.joint_pos_rel_without_wheel, which returns ALL 16 joint
    // positions (legs + wheels) with the 4 wheel slots zeroed -- it does NOT
    // drop them. So the policy's joint_pos observation is 16-dim (12 leg
    // positions followed by 4 zero wheel slots, matching joint_names =
    // leg_joint_names + wheel_joint_names). Appending the zeroed wheel slots
    // here makes the observation 57-wide, which is the exported model's input.
    Eigen::VectorXd joint_positions_with_wheels(16);
    joint_positions_with_wheels << leg_joint_positions, Eigen::Vector4d::Zero();

    if (initialized_) {
      prev_action_.setZero(16);
      initialized_ = false;
    } else {
      prev_action_ = raw_actions_;
    }

    obs_.resize(57);
    obs_ << base_ang_vel, proj_gravity, vel_cmd, joint_positions_with_wheels,
        joint_velocities_with_wheels, prev_action_;
    return;
  }

  Eigen::VectorXd temp_joint_positions(3 * num_feet_),
      temp_joint_velocities(3 * num_feet_);

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

  // Apply Scaling Factors, Make Joint Positions Relative to Base Stance Pose
  joint_positions -= nominal_stance_pose_;
  base_ang_vel *= 0.2;
  joint_velocities *= 0.05;

  // Compute Prev Action
  if (initialized_) {
    prev_action_ = (joint_positions - nominal_stance_pose_) / scale_factor_;
    initialized_ = false;
  } else {
    prev_action_ = raw_actions_;
  }

  obs_.resize(45);
  obs_ << base_ang_vel, proj_gravity, vel_cmd, joint_positions,
      joint_velocities, prev_action_;
}

void LearnedPolicy::runInference() {
  // Run Inference and Update Previous Action
  if (!session_) {
    RCLCPP_INFO(node_->get_logger(), "ONNX Session Not Ready");
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

  // Guard the inference call: a wrong observation size throws an Ort exception.
  // Catching it here logs the mismatch instead of crashing the whole
  // robot_driver node (which silently leaves the robot unresponsive).
  std::vector<Ort::Value> outs;
  try {
    outs = session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor, 1,
                         out_names, 1);
  } catch (const Ort::Exception& e) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "ONNX inference failed (obs size = %ld): %s. Check the observation "
        "matches the policy's expected input width.",
        (long)obs_f.size(), e.what());
    return;
  }

  if (outs.size() != 1 || !outs[0].IsTensor()) {
    RCLCPP_ERROR(node_->get_logger(), "Unexpected ONNX output");
    return;
  }

  auto out_info = outs[0].GetTensorTypeAndShapeInfo();
  auto out_shape = out_info.GetShape();

  int64_t out_elems = 1;
  for (auto d : out_shape) out_elems *= (d < 0 ? 1 : d);

  float* y = outs[0].GetTensorMutableData<float>();
  raw_actions_.resize(out_elems);
  for (int64_t i = 0; i < out_elems; ++i)
    raw_actions_(i) = static_cast<double>(y[i]);

  const int expected_action_size = is_go2w_policy_ ? 16 : actions_.size();
  if (raw_actions_.size() < expected_action_size) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Policy output has %ld actions, expected at least %d",
                 raw_actions_.size(), expected_action_size);
    return;
  }

  if (raw_actions_.size() > expected_action_size && !warned_extra_actions_) {
    RCLCPP_WARN(node_->get_logger(),
                "Policy output has %ld actions; using first %d actions",
                raw_actions_.size(), expected_action_size);
    warned_extra_actions_ = true;
  }

  if (is_go2w_policy_) {
    // RobotLab Go2-W action order:
    //   leg positions: FR, FL, RR, RL (hip, thigh, calf per leg)
    //   wheel velocities: FR, FL, RR, RL
    Eigen::VectorXd robotlab_leg_actions = raw_actions_.head(12);
    Eigen::VectorXd robotlab_nominal_stance(12);
    robotlab_nominal_stance << 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8,
        -1.5, 0.0, 0.8, -1.5;

    Eigen::VectorXd robotlab_leg_targets(12);
    for (int leg = 0; leg < 4; ++leg) {
      const int base = 3 * leg;
      robotlab_leg_targets(base + 0) =
          robotlab_leg_actions(base + 0) * 0.125 + robotlab_nominal_stance(base + 0);
      robotlab_leg_targets(base + 1) =
          robotlab_leg_actions(base + 1) * 0.25 + robotlab_nominal_stance(base + 1);
      robotlab_leg_targets(base + 2) =
          robotlab_leg_actions(base + 2) * 0.25 + robotlab_nominal_stance(base + 2);
    }

    actions_ << robotlab_leg_targets(3), robotlab_leg_targets(4),
        robotlab_leg_targets(5), robotlab_leg_targets(9),
        robotlab_leg_targets(10), robotlab_leg_targets(11),
        robotlab_leg_targets(0), robotlab_leg_targets(1),
        robotlab_leg_targets(2), robotlab_leg_targets(6),
        robotlab_leg_targets(7), robotlab_leg_targets(8);
    for (int leg = 0; leg < 4; ++leg) {
      actions_(3 * leg + 1) = robotLabHipToQuadGo2w(actions_(3 * leg + 1));
      actions_(3 * leg + 2) = robotLabKneeToQuadGo2w(actions_(3 * leg + 2));
    }

    wheel_actions_ << raw_actions_(13) * 5.0, raw_actions_(15) * 5.0,
        raw_actions_(12) * 5.0, raw_actions_(14) * 5.0;
    return;
  }

  // Apply Scaling to Raw Action Offsets and Add Them to Nominal Stance
  Eigen::VectorXd unordered_actions_ =
      raw_actions_.head(actions_.size()) * scale_factor_ + nominal_stance_pose_;

  // Reorder Actions to Match the Quad-SDK Joint Convention
  // Quad SDK Convention FL(8, 0, 1), RL(9, 2, 3), FR(10, 4, 5), RR(11, 6, 7)

  // Isaac Lab Action Output Convention
  // (FL hip 0, FR hip 1, RL hip 2, RR hip 3), (FL thigh 4, FR thigh 5, RL thigh
  // 6, RR thigh 7), (FL calf 8, FR calf 9, RL calf 10, RR calf 11)
  actions_ << unordered_actions_(0), unordered_actions_(4),
      unordered_actions_(8), unordered_actions_(2), unordered_actions_(6),
      unordered_actions_(10), unordered_actions_(1), unordered_actions_(5),
      unordered_actions_(9), unordered_actions_(3), unordered_actions_(7),
      unordered_actions_(11);

  // Print out Action Commands as a Debugging Step
  // temp_actions_ << 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8,
  //     -1.5;
  // std::cout << "Outputted Actions"  << unordered_actions_ -
  // nominal_stance_pose_ << std::endl;
}

bool LearnedPolicy::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
  // Safety: return false if cmd_vel is stale
  const double cmd_vel_age = (node_->now() - last_cmd_vel_msg_time_).seconds();
  if (cmd_vel_age >= kCmdVelTimeoutSec) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[LearnedPolicy] cmd_vel STALE (age=%.2fs >= %.2fs) -> holding stand "
        "pose, joystick ignored. Is %s being published?",
        cmd_vel_age, kCmdVelTimeoutSec, "cmd_vel");
    return false;
  }

  // Run inference at policy_inference_rate_ (e.g. 50 Hz), not every tick
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
    leg.motor_commands.resize(is_go2w_policy_ ? 4 : 3);

    for (int j = 0; j < 3; ++j) {
      const int idx = 3 * i + j;
      auto& cmd = leg.motor_commands.at(j);

      // Position targets from last inference, tracked by PD in robot_driver
      cmd.pos_setpoint = actions_(idx);
      cmd.vel_setpoint = 0.0;
      cmd.torque_ff = 0.0;

      cmd.kp = stance_kp_.at(j);
      cmd.kd = stance_kd_.at(j);
    }

    if (is_go2w_policy_) {
      auto& wheel_cmd = leg.motor_commands.at(3);
      wheel_cmd.pos_setpoint = 0.0;
      wheel_cmd.vel_setpoint = wheel_actions_(i);
      wheel_cmd.torque_ff = 0.0;
      wheel_cmd.kp = 0.0;
      wheel_cmd.kd = 0.5;
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
