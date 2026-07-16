#include "robot_driver/controllers/underbrush_policy.hpp"

UnderbrushPolicy::UnderbrushPolicy(rclcpp::Node::SharedPtr node,
                                   const std::string& robot_ns,
                                   std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LearnedVelocityPolicy(node, robot_ns, quadKD) {
  // Underbrush actor was trained with an action scale of 0.5 (the base MLP
  // policy uses 0.25). postProcessActions() reads scale_factor_, so overriding
  // it here is all that is needed to reuse the shared action head.
  scale_factor_ = 0.5;

  // Allocate all buffers up front to avoid per-inference allocations.
  for (int leg = 0; leg < 4; ++leg) {
    obs_per_leg_[leg].assign(kPerLegObsDim, 0.0f);
    h_state_[leg].assign(kGRUNumLayers * kBatch * kGRUHidden, 0.0f);
  }
  obs_body_.assign(kBodyObsDim, 0.0f);
}

void UnderbrushPolicy::resetHiddenStates() {
  for (auto& h : h_state_) {
    std::fill(h.begin(), h.end(), 0.0f);
  }
  RCLCPP_INFO(node_->get_logger(),
              "UnderbrushPolicy: per-leg GRU hidden states reset");
}

void UnderbrushPolicy::updateFootContactMsg(
    const quad_msgs::msg::FootContact& msg) {
  last_foot_contact_msg_ = msg;
}

void UnderbrushPolicy::computeObservations(
    const quad_msgs::msg::RobotState& robot_state_msg) {
  // robot_state joints/effort are in Quad-SDK order (leg-then-joint:
  // FL,RL,FR,RR × hip,thigh,knee). The GRU expects per-leg groups in Isaac
  // leg order (FL,FR,RL,RR); kQuadLegOfIsaac maps between them.
  const auto& q_raw = robot_state_msg.joints.position;
  const auto& qd_raw = robot_state_msg.joints.velocity;
  const auto& tau_raw = robot_state_msg.joints.effort;
  const auto& foot_force_raw = last_foot_contact_msg_.foot_force_raw;

  // last_action for the body obs is the previous inference's raw action (Isaac
  // order). raw_actions_ starts at zero, so the first step sees zeros — the
  // standard IsaacLab convention.
  prev_action_ = raw_actions_;

  // ------------------------------------------------------------------
  // Body observation (21 dims)
  //
  // Layout must match VineWalkV28GRUObservationsCfg._BodyObsCfg:
  //   base_ang_vel(3, scale=0.2) + projected_gravity(3) +
  //   velocity_commands(3) + last_action(12)
  // ------------------------------------------------------------------
  int idx = 0;

  // base_ang_vel — from IMU angular velocity, scaled to match the sim's
  // ObsTerm(scale=0.2) on base_ang_vel.
  obs_body_[idx++] =
      kAngVelScale * static_cast<float>(last_imu_msg_.angular_velocity.x);
  obs_body_[idx++] =
      kAngVelScale * static_cast<float>(last_imu_msg_.angular_velocity.y);
  obs_body_[idx++] =
      kAngVelScale * static_cast<float>(last_imu_msg_.angular_velocity.z);

  // projected_gravity = quat_rotate_inverse(q_world_body, (0,0,-1)), i.e.
  // R_body_world · (0,0,-1). Expanded directly from the body orientation
  // quaternion in robot_state_msg.body.pose.orientation.
  {
    const auto& q = robot_state_msg.body.pose.orientation;
    const double gx = -2.0 * (q.x * q.z - q.w * q.y);
    const double gy = -2.0 * (q.y * q.z + q.w * q.x);
    const double gz = -(q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    obs_body_[idx++] = static_cast<float>(gx);
    obs_body_[idx++] = static_cast<float>(gy);
    obs_body_[idx++] = static_cast<float>(gz);
  }

  // velocity_commands (vx, vy, wz) — clipped to the trained bounds. cmd_vel_
  // is the 6-vector [vx,vy,vz,wx,wy,wz]; yaw rate lives at index 5.
  {
    const double wz = (cmd_vel_msg_.size() > 5) ? cmd_vel_msg_(5) : 0.0;
    Eigen::Vector3d vel_cmd(cmd_vel_msg_(0), cmd_vel_msg_(1), wz);
    const Eigen::Vector3d vmin(-1.0, -0.4, -1.0);
    const Eigen::Vector3d vmax(1.0, 0.4, 1.0);
    vel_cmd = vel_cmd.cwiseMin(vmax).cwiseMax(vmin);
    obs_body_[idx++] = static_cast<float>(vel_cmd(0));
    obs_body_[idx++] = static_cast<float>(vel_cmd(1));
    obs_body_[idx++] = static_cast<float>(vel_cmd(2));
  }

  // last_action (12) — previous raw action, already in Isaac joint order
  for (int i = 0; i < 12; ++i) {
    obs_body_[idx++] = static_cast<float>(prev_action_(i));
  }

  // ------------------------------------------------------------------
  // Per-leg observations (10 dims each)
  //
  // Layout must match VineWalkV28GRUObservationsCfg._make_leg_obs_cfg:
  //   q_leg(3) + qd_leg(3) + foot_force(1) + tau_meas_leg(3)
  // Loop iterates legs in Isaac order (FL,FR,RL,RR); each leg's joints are
  // pulled from their Quad-SDK slot (3*quad_leg + joint).
  // ------------------------------------------------------------------
  const bool joints_ok = q_raw.size() >= 12 && qd_raw.size() >= 12 &&
                         tau_raw.size() >= 12;
  for (int leg = 0; leg < 4; ++leg) {
    auto& obs = obs_per_leg_[leg];
    const int quad_leg = kQuadLegOfIsaac[leg];
    int k = 0;

    // q_leg (hip, thigh, calf) — absolute joint positions
    for (int j = 0; j < 3; ++j) {
      const int raw_idx = 3 * quad_leg + j;
      obs[k++] = joints_ok
                     ? kJointPosScale * static_cast<float>(q_raw.at(raw_idx))
                     : 0.0f;
    }
    // qd_leg
    for (int j = 0; j < 3; ++j) {
      const int raw_idx = 3 * quad_leg + j;
      obs[k++] = joints_ok
                     ? kJointVelScale * static_cast<float>(qd_raw.at(raw_idx))
                     : 0.0f;
    }
    // foot_force — raw Unitree count → Newtons (calibration) → sim obs scale.
    // foot_force_raw is Quad-SDK ordered, so index by quad_leg.
    const float ff =
        (static_cast<int>(foot_force_raw.size()) > quad_leg)
            ? static_cast<float>(foot_force_raw[quad_leg])
            : 0.0f;
    obs[k++] = ff * kFootForceCountsToNewtons * kFootForceObsScale;

    // tau_meas_leg — measured joint torque (Unitree tau_est, already N·m)
    for (int j = 0; j < 3; ++j) {
      const int raw_idx = 3 * quad_leg + j;
      obs[k++] = joints_ok
                     ? kTauScale * static_cast<float>(tau_raw.at(raw_idx))
                     : 0.0f;
    }
  }
}

void UnderbrushPolicy::runInference() {
  if (!session_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "UnderbrushPolicy: ONNX session not initialized");
    return;
  }

  // Populate the ONNX I/O name cache the first time this runs. Names are
  // baked into the session by export_underbrush_onnx.py.
  if (input_names_.empty()) {
    Ort::AllocatorWithDefaultOptions alloc;
    const size_t n_in = session_->GetInputCount();
    const size_t n_out = session_->GetOutputCount();

    if (n_in != 9 || n_out != 5) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "UnderbrushPolicy expects 9 inputs + 5 outputs; got %zu + %zu. "
          "Did you export a per-leg-GRU checkpoint with "
          "export_underbrush_onnx.py --actor-type gru ?",
          n_in, n_out);
      return;
    }

    input_names_.reserve(n_in);
    output_names_.reserve(n_out);
    in_name_cstrs_.reserve(n_in);
    out_name_cstrs_.reserve(n_out);
    for (size_t i = 0; i < n_in; ++i) {
      auto n = session_->GetInputNameAllocated(i, alloc);
      input_names_.emplace_back(n.get());
      in_name_cstrs_.push_back(input_names_.back().c_str());
    }
    for (size_t i = 0; i < n_out; ++i) {
      auto n = session_->GetOutputNameAllocated(i, alloc);
      output_names_.emplace_back(n.get());
      out_name_cstrs_.push_back(output_names_.back().c_str());
    }
  }

  // Build input tensors in the exact order the ONNX wrapper expects.
  // From export_underbrush_onnx.py PerLegGRUONNXWrapper.forward():
  //   (per_leg_FL, per_leg_FR, per_leg_RL, per_leg_RR,
  //    body,
  //    h_FL, h_FR, h_RL, h_RR)
  std::vector<Ort::Value> inputs;
  inputs.reserve(9);

  const int64_t per_leg_shape[] = {kBatch, kPerLegObsDim};
  const int64_t body_shape[] = {kBatch, kBodyObsDim};
  const int64_t hidden_shape[] = {kGRUNumLayers, kBatch, kGRUHidden};

  for (int leg = 0; leg < 4; ++leg) {
    inputs.push_back(Ort::Value::CreateTensor<float>(
        mem_info_, obs_per_leg_[leg].data(), obs_per_leg_[leg].size(),
        per_leg_shape, 2));
  }
  inputs.push_back(Ort::Value::CreateTensor<float>(
      mem_info_, obs_body_.data(), obs_body_.size(), body_shape, 2));
  for (int leg = 0; leg < 4; ++leg) {
    inputs.push_back(
        Ort::Value::CreateTensor<float>(mem_info_, h_state_[leg].data(),
                                        h_state_[leg].size(), hidden_shape, 3));
  }

  // Run: 9 inputs → 5 outputs (actions + 4 h_out)
  auto outs = session_->Run(Ort::RunOptions{nullptr}, in_name_cstrs_.data(),
                            inputs.data(), 9, out_name_cstrs_.data(), 5);

  if (outs.size() != 5) {
    RCLCPP_ERROR(node_->get_logger(),
                 "UnderbrushPolicy: unexpected output count: %zu", outs.size());
    return;
  }

  // Output 0: actions — shape [1, 12]
  {
    float* y = outs[0].GetTensorMutableData<float>();
    raw_actions_.resize(kActionDim);
    for (int i = 0; i < kActionDim; ++i) {
      raw_actions_(i) = static_cast<double>(y[i]);
    }
  }

  // Outputs 1-4: updated hidden states — copy back into h_state_ for next call
  for (int leg = 0; leg < 4; ++leg) {
    float* h_out = outs[1 + leg].GetTensorMutableData<float>();
    std::copy(h_out, h_out + h_state_[leg].size(), h_state_[leg].data());
  }

  // Scale + nominal offset + Isaac→Quad-SDK reorder into actions_ (shared with
  // the base MLP policy; scale_factor_ was set to 0.5 in the constructor).
  postProcessActions();
}
