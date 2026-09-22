#include "robot_driver/controllers/underbrush_policy.hpp"
#include <stdexcept>

UnderbrushPolicy::UnderbrushPolicy(rclcpp::Node::SharedPtr node,
                                   const std::string& robot_ns,
                                   std::shared_ptr<quad_utils::QuadKD2> quadKD,
                                   bool per_leg_action_history)
    : LearnedVelocityPolicy(node, robot_ns, quadKD),
      per_leg_obs_dim_(per_leg_action_history ? 13 : kPerLegObsDim) {
  // NB: scale_factor_, stance_kp/kd, and nominal_stance_pose_ are all set in
  // our overridden init() below — NOT here. Setting them in the ctor gets
  // clobbered by LearnedVelocityPolicy::init() which RobotDriver calls after
  // construction to populate PD gains + nominal stance from robot_driver.yaml.

  // Allocate all buffers up front to avoid per-inference allocations.
  for (int leg = 0; leg < 4; ++leg) {
    obs_per_leg_[leg].assign(per_leg_obs_dim_, 0.0f);
    h_state_[leg].assign(kGRUNumLayers * kBatch * kGRUHidden, 0.0f);
  }
  obs_body_.assign(kBodyObsDim, 0.0f);
  obs_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("policy/observations", 10);
  action_debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("policy/raw_actions", 10);

  // CRITICAL: pre-size raw_actions_ and prev_action_ so tick-0's computeObs
  // doesn't OOB-access. Base class does NOT init them, and runInference only
  // resizes after the first ONNX call — but computeObs runs BEFORE inference
  // and reads prev_action_(i) for i=0..11 into the last_action obs slot. UB
  // last_action makes the GRU output garbage on tick 0, and the recurrent
  // state carries that contamination forward → the policy diverges instantly.
  raw_actions_ = Eigen::VectorXd::Zero(kActionDim);
  prev_action_ = Eigen::VectorXd::Zero(kActionDim);
}

void UnderbrushPolicy::init(
    const std::vector<double>& stance_kp, const std::vector<double>& stance_kd,
    const std::vector<double>& swing_kp, const std::vector<double>& swing_kd,
    const std::vector<double>& swing_kp_cart,
    const std::vector<double>& swing_kd_cart, const std::string& model_path,
    double policy_inference_rate,
    const std::vector<double>& stand_joint_angles) {
  // Let the parent do the heavy lifting (kp/kd store, model load, uniform
  // nominal_stance_pose_ from stand_joint_angles, etc.) then override the
  // v51-specific bits it clobbered.
  LearnedVelocityPolicy::init(stance_kp, stance_kd, swing_kp, swing_kd,
                              swing_kp_cart, swing_kd_cart, model_path,
                              policy_inference_rate, stand_joint_angles);

  // Validate before any control step; V90 and V92 are not interchangeable.
  if (!session_ || session_->GetInputCount() != 9 || session_->GetOutputCount() != 5) {
    throw std::runtime_error("Underbrush requires a 9-input, 5-output GRU model");
  }
  // Self-configure the per-leg width from the loaded model. V121-class
  // exports take 9-dim per-leg inputs (no binary foot-contact channel);
  // V90/V115-class take 10 (V92: 13). Everything downstream (validation,
  // buffers, tensor shapes, obs assembly) follows leg_obs_dim_.
  leg_obs_dim_ = per_leg_obs_dim_;
  include_contact_ = true;
  {
    auto leg0 = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
    const auto leg0_shape = leg0.GetShape();
    if (per_leg_obs_dim_ == kPerLegObsDim && !leg0_shape.empty() &&
        leg0_shape.back() == kPerLegObsDim - 1) {
      leg_obs_dim_ = kPerLegObsDim - 1;
      include_contact_ = false;
      for (auto& leg : obs_per_leg_) leg.assign(leg_obs_dim_, 0.0f);
      RCLCPP_INFO(node_->get_logger(),
                  "UnderbrushPolicy: contactless model detected (per-leg 9 "
                  "dims) — binary foot-contact observation disabled");
    }
  }
  Ort::AllocatorWithDefaultOptions alloc;
  const std::vector<std::string> expected_inputs = {
      "per_leg_FL", "per_leg_FR", "per_leg_RL", "per_leg_RR", "body",
      "h_FL", "h_FR", "h_RL", "h_RR"};
  const std::vector<std::string> expected_outputs = {
      "actions", "h_FL_out", "h_FR_out", "h_RL_out", "h_RR_out"};
  for (size_t i = 0; i < 14; ++i) {
    const bool input = i < 9;
    const size_t index = input ? i : i - 9;
    auto name = input ? session_->GetInputNameAllocated(index, alloc)
                      : session_->GetOutputNameAllocated(index, alloc);
    auto info = input ? session_->GetInputTypeInfo(index)
                      : session_->GetOutputTypeInfo(index);
    auto tensor = info.GetTensorTypeAndShapeInfo();
    auto shape = tensor.GetShape();
    const bool hidden = input ? index >= 5 : index >= 1;
    const int dim = hidden ? kGRUHidden :
        (input ? (index < 4 ? leg_obs_dim_ : kBodyObsDim) : kActionDim);
    const auto& expected_name = input ? expected_inputs[index] : expected_outputs[index];
    if (name.get() != expected_name ||
        tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        shape.size() != (hidden ? 3u : 2u) || shape.back() != dim ||
        (hidden && shape[0] != kGRUNumLayers) ||
        (shape[hidden ? 1 : 0] != -1 && shape[hidden ? 1 : 0] != kBatch)) {
      throw std::runtime_error("Underbrush model contract mismatch at " + expected_name +
                               "; check controller version and export dimensions");
    }
  }

  // Exercise runtime setup before handing commands to the robot. Outputs are
  // discarded; real hidden state and previous actions still start at zero.
  std::vector<Ort::Value> preflight_inputs;
  std::vector<const char*> preflight_in_names, preflight_out_names;
  const int64_t leg_shape[] = {1, leg_obs_dim_};
  const int64_t body_shape[] = {1, kBodyObsDim};
  const int64_t hidden_shape[] = {1, 1, kGRUHidden};
  for (auto& name : expected_inputs) preflight_in_names.push_back(name.c_str());
  for (auto& name : expected_outputs) preflight_out_names.push_back(name.c_str());
  for (auto& leg : obs_per_leg_) preflight_inputs.push_back(
      Ort::Value::CreateTensor<float>(mem_info_, leg.data(), leg.size(), leg_shape, 2));
  preflight_inputs.push_back(Ort::Value::CreateTensor<float>(
      mem_info_, obs_body_.data(), obs_body_.size(), body_shape, 2));
  for (auto& hidden : h_state_) preflight_inputs.push_back(
      Ort::Value::CreateTensor<float>(mem_info_, hidden.data(), hidden.size(), hidden_shape, 3));
  session_->Run(Ort::RunOptions{nullptr}, preflight_in_names.data(),
                preflight_inputs.data(), 9, preflight_out_names.data(), 5);

  // v51 training used IsaacLab Go2 defaults: stiffness=25.0, damping=0.5 on
  // all leg joints, actuator DR ±20%. The base go2.yaml stance_kp/kd of 60/4
  // is calibrated for the MPC and is far outside training's range.
  const std::vector<double> v51_kp = {25.0, 25.0, 25.0};
  const std::vector<double> v51_kd = {0.5, 0.5, 0.5};
  stance_kp_ = v51_kp;
  stance_kd_ = v51_kd;
  swing_kp_ = v51_kp;
  swing_kd_ = v51_kd;

  // Keep the nominal pose loaded by the base class from stand_joint_angles.
  // V90 training overrides the asset defaults to [0.0, 0.8, -1.5]
  // on every leg; this pose is used for both observations and action offsets.

  scale_factor_ = 0.5;  // v51 action_scale (from v43 lineage)

  RCLCPP_INFO(node_->get_logger(),
              "UnderbrushPolicy::init overrides: kp=25 kd=0.5, "
              "YAML nominal stance, action_scale=0.5; "
              "torque observation=RobotState joints.effort, scale=0.01");
}

void UnderbrushPolicy::resetHiddenStates() {
  for (auto& h : h_state_) {
    std::fill(h.begin(), h.end(), 0.0f);
  }
  warmup_ticks_remaining_ = kEncoderWarmupTicks;
  RCLCPP_INFO(node_->get_logger(),
              "UnderbrushPolicy: per-leg GRU hidden states reset; "
              "encoder-warmup armed for %d ticks",
              kEncoderWarmupTicks);
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
  const auto& contact = last_foot_contact_msg_.contact_states;

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

  // base_ang_vel — Isaac's ObsTerm reads `articulation.data.root_ang_vel_b`
  // (angular velocity in BODY frame). RobotState.body.twist.angular is body
  // frame in sim (Gazebo estimator_plugin.cpp:328-330 stores q_bw * w_w). On
  // hardware it's also body-frame gyro. `last_imu_msg_.angular_velocity` was
  // the original source here — but in the sim path `updateImuMsg` is only
  // called inside the `is_hardware_` branch of robot_driver::updateState, so
  // `last_imu_msg_` was stuck at zero every tick in Gazebo. Every previous
  // Gazebo deploy was silently feeding the policy zero angular velocity — a
  // huge missing proprioceptive channel.
  //
  // Flip the flag below to `false` to fall back to IMU (matches HW convention;
  // works on the physical robot, breaks in Gazebo unless someone plumbs
  // updateImuMsg in the sim path too).
  constexpr bool kAngVelFromRobotState = true;
  if (kAngVelFromRobotState) {
    const auto& w = robot_state_msg.body.twist.angular;
    obs_body_[idx++] = kAngVelScale * static_cast<float>(w.x);
    obs_body_[idx++] = kAngVelScale * static_cast<float>(w.y);
    obs_body_[idx++] = kAngVelScale * static_cast<float>(w.z);
  } else {
    obs_body_[idx++] = kAngVelScale * static_cast<float>(last_imu_msg_.angular_velocity.x);
    obs_body_[idx++] = kAngVelScale * static_cast<float>(last_imu_msg_.angular_velocity.y);
    obs_body_[idx++] = kAngVelScale * static_cast<float>(last_imu_msg_.angular_velocity.z);
  }

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
  // Layout must match v51's _make_leg_obs_cfg term order (concatenate_terms):
  //   joint_pos_rel(3) + joint_vel_rel(3) + joint_effort(3) + foot_force(1)
  // Loop iterates legs in Isaac order (FL,FR,RL,RR); each leg's joints are
  // pulled from their Quad-SDK slot (3*quad_leg + joint).
  // ------------------------------------------------------------------
  const bool joints_ok =
      q_raw.size() >= 12 && qd_raw.size() >= 12;
  for (int leg = 0; leg < 4; ++leg) {
    auto& obs = obs_per_leg_[leg];
    const int quad_leg = kQuadLegOfIsaac[leg];
    int k = 0;

    // q_leg — joint_pos_rel: (q - default), scale 1.0.
    // nominal_stance_pose_ is in Isaac's BY-JOINT-TYPE layout:
    //   [FL_abd, FR_abd, RL_abd, RR_abd,
    //    FL_thigh, FR_thigh, RL_thigh, RR_thigh,
    //    FL_calf, FR_calf, RL_calf, RR_calf]
    // so joint j (0=abd/hip, 1=thigh, 2=calf) of Isaac leg 'leg' (FL=0, FR=1,
    // RL=2, RR=3) is at index (4*j + leg). Using nominal(4*j) alone grabs
    // FL's value for every leg — WRONG for Isaac's asymmetric defaults
    // (L/R abads differ, F/R thighs differ).
    for (int j = 0; j < 3; ++j) {
      const int raw_idx = 3 * quad_leg + j;
      obs[k++] = joints_ok ? kJointPosScale *
                                 static_cast<float>(q_raw.at(raw_idx) -
                                                    nominal_stance_pose_(4 * j + leg))
                           : 0.0f;
    }
    // qd_leg — joint_vel_rel (default vel = 0 → absolute), scale 0.05
    for (int j = 0; j < 3; ++j) {
      const int raw_idx = 3 * quad_leg + j;
      obs[k++] = joints_ok
                     ? kJointVelScale * static_cast<float>(qd_raw.at(raw_idx))
                     : 0.0f;
    }
    // RobotDriver supplies simulator effort when is_hardware=false and
    // Unitree tau_est when true. Both arrive in Quad-SDK joint order.
    // Use the received signal even at the first inference (no PD proxy).
    for (int j = 0; j < 3; ++j) {
      const int raw_idx = 3 * quad_leg + j;
      obs[k++] = tau_raw.size() >= 12
          ? kTauScale * static_cast<float>(tau_raw.at(raw_idx)) : 0.0f;
    }
    // foot_force — BINARY contact (v51 binarizes at 5 N, scale 1.0).
    //
    // On HARDWARE: robot_driver::updateState() calls
    // hardware_interface_->getFootContact(...) → updateFootContactMsg on us.
    // In SIM: RobotDriver::simGrfsCallback binarizes topics.state.grfs (the
    // Gazebo ground-truth per-foot GRFs) and pushes the result the same way.
    // If neither has fired yet (first tick), fall back to 0 (airborne) rather
    // than 1 — a stray "planted" reading during ballistic drop before the
    // subscribers wake up caused the policy to command a full-weight push.
    if (include_contact_) {
      if (static_cast<int>(contact.size()) >= 4) {
        obs[k++] = contact[quad_leg] ? 1.0f : 0.0f;
      } else {
        obs[k++] = 0.0f;
      }
    }
    // V92 appends this leg's previous raw hip/thigh/calf actions. The
    // action vector is in Isaac joint-type order, not Quad-SDK leg order.
    if (per_leg_obs_dim_ == 13) {
      for (int j = 0; j < 3; ++j) {
        obs[k++] = static_cast<float>(prev_action_(4 * j + leg));
      }
    }
  }

  // Diagnostic order: body (21), then FL/FR/RL/RR legs (10 or 13 each).
  std_msgs::msg::Float32MultiArray observation_debug;
  observation_debug.data = obs_body_;
  for (const auto& leg : obs_per_leg_)
    observation_debug.data.insert(observation_debug.data.end(), leg.begin(), leg.end());
  obs_debug_pub_->publish(observation_debug);

  // -------- DEBUG: first 5 ticks dump obs to verify sim-to-deploy alignment --------
  static int dbg_tick = 0;
  if (dbg_tick < 5) {
    const auto& q = robot_state_msg.body.pose.orientation;
    RCLCPP_INFO(node_->get_logger(),
      "[dbg %d] quat(w,x,y,z)=(%.3f,%.3f,%.3f,%.3f)  proj_g=(%.3f,%.3f,%.3f)  "
      "ang_vel=(%.3f,%.3f,%.3f)  vel_cmd=(%.3f,%.3f,%.3f)",
      dbg_tick, q.w, q.x, q.y, q.z,
      obs_body_[3], obs_body_[4], obs_body_[5],
      obs_body_[0]/kAngVelScale, obs_body_[1]/kAngVelScale, obs_body_[2]/kAngVelScale,
      obs_body_[6], obs_body_[7], obs_body_[8]);
    // Contact source — this is the user's concern: is last_foot_contact_msg_
    // actually being populated in the Gazebo sim path?
    RCLCPP_INFO(node_->get_logger(),
      "[dbg %d] contact_states.size()=%zu  values=[%s,%s,%s,%s]  header.frame=%s",
      dbg_tick, contact.size(),
      (contact.size() > 0 && contact[0]) ? "1" : "0",
      (contact.size() > 1 && contact[1]) ? "1" : "0",
      (contact.size() > 2 && contact[2]) ? "1" : "0",
      (contact.size() > 3 && contact[3]) ? "1" : "0",
      last_foot_contact_msg_.header.frame_id.c_str());
    // per-leg raw q vs Isaac nominal — should be near-zero q_rel if Gazebo init matches training
    const char* leg_names[4] = {"FL", "FR", "RL", "RR"};
    for (int leg = 0; leg < 4; ++leg) {
      const int quad_leg = kQuadLegOfIsaac[leg];
      const double q_abd  = joints_ok ? q_raw.at(3 * quad_leg + 0) : 0.0;
      const double q_thg  = joints_ok ? q_raw.at(3 * quad_leg + 1) : 0.0;
      const double q_calf = joints_ok ? q_raw.at(3 * quad_leg + 2) : 0.0;
      const double n_abd  = nominal_stance_pose_(4 * 0 + leg);
      const double n_thg  = nominal_stance_pose_(4 * 1 + leg);
      const double n_calf = nominal_stance_pose_(4 * 2 + leg);
      RCLCPP_INFO(node_->get_logger(),
        "[dbg %d] %s q_raw=(%.3f,%.3f,%.3f) nominal=(%.3f,%.3f,%.3f) q_rel=(%.3f,%.3f,%.3f) contact_bit=%.0f",
        dbg_tick, leg_names[leg],
        q_abd, q_thg, q_calf,
        n_abd, n_thg, n_calf,
        q_abd - n_abd, q_thg - n_thg, q_calf - n_calf,
        obs_per_leg_[leg][9]);
      // Full 10-dim per-leg obs the encoder sees (q_pos:3, qd:3, tau:3, contact:1)
      RCLCPP_INFO(node_->get_logger(),
        "[dbg %d] %s obs=[qp:%.3f,%.3f,%.3f  qd:%.3f,%.3f,%.3f  tau:%.3f,%.3f,%.3f  c:%.0f]",
        dbg_tick, leg_names[leg],
        obs_per_leg_[leg][0], obs_per_leg_[leg][1], obs_per_leg_[leg][2],
        obs_per_leg_[leg][3], obs_per_leg_[leg][4], obs_per_leg_[leg][5],
        obs_per_leg_[leg][6], obs_per_leg_[leg][7], obs_per_leg_[leg][8],
        obs_per_leg_[leg][9]);
    }
    // Full body obs (21 dims): ang_vel(3), proj_g(3), vel_cmd(3), last_action(12)
    RCLCPP_INFO(node_->get_logger(),
      "[dbg %d] body last_action[0..11]=[%.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f]",
      dbg_tick,
      obs_body_[9],  obs_body_[10], obs_body_[11], obs_body_[12],
      obs_body_[13], obs_body_[14], obs_body_[15], obs_body_[16],
      obs_body_[17], obs_body_[18], obs_body_[19], obs_body_[20]);
    dbg_tick++;
  }
  // -------- END DEBUG --------
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

  const int64_t per_leg_shape[] = {kBatch, leg_obs_dim_};
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

  // cmd_vel gate: whenever operator commands near-zero motion, override the
  // MLP output to zero so postProcessActions yields actions = nominal_stance.
  // Suppresses highstepping on stand-command since v51/v65 policies default
  // to trot gait regardless of cmd_vel. Encoder still runs so hidden stays
  // warm for when cmd_vel resumes. Deploy-fix — bag from 2026-08-12_1419
  // showed 0.8m drift + persistent highstep at cmd=0.
  {
    constexpr double kStandCmdThreshold = 0.05;  // m/s + rad/s
    double lin_mag = std::hypot(cmd_vel_msg_(0), cmd_vel_msg_(1));
    // cmd_vel_ uses Twist ordering [vx, vy, vz, wx, wy, wz].  Yaw rate is
    // index 5; index 2 is linear-z and is always zeroed by cmdVelCallback().
    double ang_mag = std::abs(cmd_vel_msg_(5));
    if (lin_mag < kStandCmdThreshold && ang_mag < kStandCmdThreshold) {
      raw_actions_.setZero();
    }
  }

  // Encoder-warmup: for kEncoderWarmupTicks after any reset, we let the
  // encoders run forward (hidden state updates above) but zero-out the raw
  // MLP output so postProcessActions produces actions = nominal_stance
  // (raw*scale + nominal, with raw=0). Deploy-fix — see resetHiddenStates
  // docstring.
  if (warmup_ticks_remaining_ > 0) {
    raw_actions_.setZero();
    warmup_ticks_remaining_--;
    if (warmup_ticks_remaining_ == 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "UnderbrushPolicy: encoder-warmup complete → releasing MLP "
                  "head to actuators");
    }
  }

  // Scale + nominal offset + Isaac→Quad-SDK reorder into actions_ (shared with
  // the base MLP policy). The parent's reorder is CORRECT for Isaac's
  // by-joint-type layout (verified via live AppLauncher joint_names query).
  postProcessActions();
  std_msgs::msg::Float32MultiArray action_debug;
  for (int i = 0; i < kActionDim; ++i)
    action_debug.data.push_back(static_cast<float>(raw_actions_(i)));
  action_debug_pub_->publish(action_debug);

  // -------- DEBUG: print raw ONNX output + final actuator commands --------
  static int inf_dbg = 0;
  if (inf_dbg < 5) {
    RCLCPP_INFO(node_->get_logger(),
      "[inf %d] raw_actions (Isaac by-joint):\n"
      "  abd:   [%.3f, %.3f, %.3f, %.3f]  (FL,FR,RL,RR)\n"
      "  thigh: [%.3f, %.3f, %.3f, %.3f]\n"
      "  calf:  [%.3f, %.3f, %.3f, %.3f]",
      inf_dbg,
      raw_actions_(0), raw_actions_(1), raw_actions_(2), raw_actions_(3),
      raw_actions_(4), raw_actions_(5), raw_actions_(6), raw_actions_(7),
      raw_actions_(8), raw_actions_(9), raw_actions_(10), raw_actions_(11));
    RCLCPP_INFO(node_->get_logger(),
      "[inf %d] actions_ (Quad-SDK by-leg, sent to actuators):\n"
      "  FL[abd,hip,knee]=[%.3f, %.3f, %.3f]\n"
      "  RL[abd,hip,knee]=[%.3f, %.3f, %.3f]\n"
      "  FR[abd,hip,knee]=[%.3f, %.3f, %.3f]\n"
      "  RR[abd,hip,knee]=[%.3f, %.3f, %.3f]",
      inf_dbg,
      actions_(0), actions_(1), actions_(2),
      actions_(3), actions_(4), actions_(5),
      actions_(6), actions_(7), actions_(8),
      actions_(9), actions_(10), actions_(11));
    inf_dbg++;
  }
  // -------- END DEBUG --------
}
