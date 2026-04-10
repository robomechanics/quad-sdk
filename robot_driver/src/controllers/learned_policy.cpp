#include "robot_driver/controllers/learned_policy.hpp"

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
                         double policy_inference_rate) {
  // Initalize the Path to the Model Onnx File
  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
  swing_kp_cart_ = swing_kp_cart;
  swing_kd_cart_ = swing_kd_cart;
  model_path_ = model_path;
  policy_inference_rate_ = policy_inference_rate;
  first_inference_ = true;
  loadONNXModel();
  // Default pose in policy order: FL, FR, RL, RR (hip, thigh, calf per leg)
  nominal_stance_pose_ << 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5,
      0.0, 0.8, -1.5;
  last_cmd_vel_msg_time_ = node_->now();
  last_inference_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
              "Loaded Learned Policy at %s (%.1f Hz)",
              model_path_.c_str(), policy_inference_rate_);
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
  Eigen::VectorXd raw_joint_positions(3 * num_feet_),
      raw_joint_velocities(3 * num_feet_), body_state(12);

  quad_utils::vectorToEigen(robot_state_msg.joints.position,
                            raw_joint_positions);
  quad_utils::vectorToEigen(robot_state_msg.joints.velocity,
                            raw_joint_velocities);
  body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg.body);

  // bodyStateMsgToEigen layout (verified in quad_utils/src/ros_utils.cpp):
  //   [0:3] position (x,y,z), [3:6] RPY, [6:9] lin_vel world-frame,
  //   [9:12] ang_vel body-frame
  //
  // Per robot_state_msg convention: linear velocity is WORLD frame,
  // angular velocity is already BODY frame — rotate only the linear part.
  const Eigen::Vector3d base_lin_vel_world(body_state(6), body_state(7),
                                           body_state(8));
  const Eigen::Vector3d base_ang_vel(body_state(9), body_state(10),
                                     body_state(11));

  // Use quaternion directly from the message (avoids re-computing from RPY)
  Eigen::Quaterniond quat(robot_state_msg.body.pose.orientation.w,
                          robot_state_msg.body.pose.orientation.x,
                          robot_state_msg.body.pose.orientation.y,
                          robot_state_msg.body.pose.orientation.z);

  // Rotate world-frame linear velocity into body frame: R^T * v_world
  const Eigen::Vector3d base_lin_vel_body = quat.conjugate() * base_lin_vel_world;

  // Projected gravity: R_base^T @ [0,0,-1] — same rotation, applied to g
  // (previously also computed via RPY rotation matrix; quaternion conjugate is
  // equivalent and avoids the intermediate R_bw construction)
  const Eigen::Vector3d g_world(0.0, 0.0, -1.0);
  const Eigen::Vector3d proj_gravity = quat.conjugate() * g_world;

  // cmd_vel_msg_ is a 6-element twist from robot_driver.cpp:
  //   [0]=vx, [1]=vy, [2]=0, [3]=0, [4]=0, [5]=yaw_rate
  Eigen::Vector3d vel_cmd(cmd_vel_msg_(0), cmd_vel_msg_(1), cmd_vel_msg_(5));
  const Eigen::Vector3d vmin(-1.0, -0.4, -1.0);
  const Eigen::Vector3d vmax(1.0, 0.4, 1.0);
  vel_cmd = vel_cmd.cwiseMin(vmax).cwiseMax(vmin);

  // SDK joint order (verified via joint_names in quad_utils/src/ros_utils.cpp):
  //   FL(0,1,2), RL(3,4,5), FR(6,7,8), RR(9,10,11)
  //
  // Policy joint order required:
  //   FL(0,1,2), FR(3,4,5), RL(6,7,8), RR(9,10,11)
  //
  // SDK_TO_POLICY remap — swap RL and FR blocks:
  //   old (IsaacLab grouped-by-type) remap was wrong for this policy
  Eigen::VectorXd joint_positions(3 * num_feet_),
      joint_velocities(3 * num_feet_);

  // clang-format off
  joint_positions <<
      raw_joint_positions(0), raw_joint_positions(1), raw_joint_positions(2),   // FL
      raw_joint_positions(6), raw_joint_positions(7), raw_joint_positions(8),   // FR (was at SDK[6:9])
      raw_joint_positions(3), raw_joint_positions(4), raw_joint_positions(5),   // RL (was at SDK[3:6])
      raw_joint_positions(9), raw_joint_positions(10), raw_joint_positions(11); // RR

  joint_velocities <<
      raw_joint_velocities(0), raw_joint_velocities(1), raw_joint_velocities(2),   // FL
      raw_joint_velocities(6), raw_joint_velocities(7), raw_joint_velocities(8),   // FR
      raw_joint_velocities(3), raw_joint_velocities(4), raw_joint_velocities(5),   // RL
      raw_joint_velocities(9), raw_joint_velocities(10), raw_joint_velocities(11); // RR
  // clang-format on

  // Offset from default pose (nominal_stance_pose_ is also in policy order)
  joint_positions -= nominal_stance_pose_;

  // No obs normalization — feed raw numbers to the policy.
  // Old code applied base_ang_vel *= 0.2 and joint_velocities *= 0.05 here;
  // those were IsaacLab-specific scalings not used by this policy.
  // base_ang_vel *= 0.2;    // REMOVED
  // joint_velocities *= 0.05;  // REMOVED

  // last_act: raw ONNX output from previous runInference() call.
  // raw_actions_ is initialized to zeros and updated after each inference,
  // so it naturally holds the previous step's output here.
  // Old logic recomputed prev_action_ from joint positions on first call
  // (initialized_ flag); that is incorrect — just use the zero-init directly.
  prev_action_ = raw_actions_;

  // 48-dim observation (no normalization):
  // [0:3]   linvel  — body-frame linear velocity
  // [3:6]   gyro    — body-frame angular velocity (already body frame)
  // [6:9]   gravity — projected gravity R^T @ [0,0,-1]
  // [9:21]  joint_pos_offset — qpos - default_pose, policy order
  // [21:33] joint_vel — policy order
  // [33:45] last_act — previous raw ONNX output (zeros on startup)
  // [45:48] command  — [vx, vy, yaw_rate]
  obs_.resize(48);
  obs_ << base_lin_vel_body, base_ang_vel, proj_gravity, joint_positions,
      joint_velocities, prev_action_, vel_cmd;

  // Old (broken) obs builds for reference:
  // obs_.resize(45);  // wrong size
  // obs_ << base_lin_vel_body, base_ang_vel, proj_gravity, vel_cmd,
  //     joint_positions, joint_velocities, prev_action_, vel_cmd;
  //     ^^^ vel_cmd duplicated and total was still wrong
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

  float* y = outs[0].GetTensorMutableData<float>();
  raw_actions_.resize(out_elems);
  for (int64_t i = 0; i < out_elems; ++i)
    raw_actions_(i) = static_cast<double>(y[i]);

  // joint_target = default_pose + scale_factor_ * raw_action
  // scale_factor_ = 0.5 (set in header); tanh is inside the ONNX graph
  // nominal_stance_pose_ is in policy order (FL, FR, RL, RR)
  Eigen::VectorXd policy_actions =
      nominal_stance_pose_ + scale_factor_ * raw_actions_;

  // POLICY_TO_SDK remap — inverse of SDK_TO_POLICY applied in computeObservations
  // Policy order: FL(0,1,2), FR(3,4,5), RL(6,7,8), RR(9,10,11)
  // SDK order:    FL(0,1,2), RL(3,4,5), FR(6,7,8), RR(9,10,11)
  // clang-format off
  actions_ <<
      policy_actions(0), policy_actions(1), policy_actions(2),    // FL -> SDK[0:3]
      policy_actions(6), policy_actions(7), policy_actions(8),    // RL -> SDK[3:6]
      policy_actions(3), policy_actions(4), policy_actions(5),    // FR -> SDK[6:9]
      policy_actions(9), policy_actions(10), policy_actions(11);  // RR -> SDK[9:12]
  // clang-format on

  // Old IsaacLab grouped-by-type remap (wrong for this policy order):
  // actions_ << unordered_actions_(0), unordered_actions_(4),
  //     unordered_actions_(8), unordered_actions_(2), unordered_actions_(6),
  //     unordered_actions_(10), unordered_actions_(1), unordered_actions_(5),
  //     unordered_actions_(9), unordered_actions_(3), unordered_actions_(7),
  //     unordered_actions_(11);

  // Print out Action Commands as a Debugging Step
  temp_actions_ << 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8, -1.5, 0.0, 0.8,
      -1.5;
  // std::cout << "Outputted Actions"  << unordered_actions_ -
  // nominal_stance_pose_ << std::endl;
}

bool LearnedPolicy::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
  // Safety: return false if cmd_vel is stale
  if ((node_->now() - last_cmd_vel_msg_time_).seconds() >= 0.1) {
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
    leg.motor_commands.resize(3);

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
  }
  return true;
}

void LearnedPolicy::updateCmdVelMsg(Eigen::VectorXd msg, rclcpp::Time& t_now) {
  cmd_vel_msg_ = msg;
  last_cmd_vel_msg_time_ = t_now;
}
