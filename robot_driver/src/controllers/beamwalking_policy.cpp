#include "robot_driver/controllers/beamwalking_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

constexpr double BeamwalkingPolicy::kSpeedRange[2];
constexpr double BeamwalkingPolicy::kDutyRange[2];
constexpr double BeamwalkingPolicy::kWidthRange[2];
constexpr int BeamwalkingPolicy::kPeriodTicksRange[2];
constexpr int BeamwalkingPolicy::kGaitQuarters[2][4];
constexpr int BeamwalkingPolicy::kQuadLegOfIsaac[4];

namespace {
// 2*(x-lo)/(hi-lo)-1, the protocol.normalized_gait_command mapping.
double normalizeCommand(double x, const double range[2]) {
  return 2.0 * (x - range[0]) / (range[1] - range[0]) - 1.0;
}

double wrapToPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}
}  // namespace

BeamwalkingPolicy::BeamwalkingPolicy(rclcpp::Node::SharedPtr node,
                                     const std::string& robot_ns,
                                     std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LearnedVelocityPolicy(node, robot_ns, quadKD) {
  // JointPositionActionCfg scale for the Go2 rough/flat env (rough_env_cfg.py
  // sets 0.25); postProcessActions() is not used here but keep the base
  // member consistent for anyone reading it.
  scale_factor_ = 0.25;
  obs_.resize(kObsDim);
  obs_.setZero();
  debug_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "beamwalking/policy_debug", 10);
}

void BeamwalkingPolicy::publishDebug(
    const quad_msgs::msg::RobotState& robot_state_msg, int tick) const {
  std_msgs::msg::Float64MultiArray msg;
  msg.data.reserve(kObsDim + 2 * kActionDim + 2);
  for (int i = 0; i < kObsDim; ++i) msg.data.push_back(obs_(i));
  for (int i = 0; i < kActionDim; ++i) msg.data.push_back(raw_actions_(i));
  for (int i = 0; i < kActionDim; ++i)
    msg.data.push_back(prev_clipped_action_(i));
  msg.data.push_back(static_cast<double>(tick));
  msg.data.push_back(rclcpp::Time(robot_state_msg.header.stamp).seconds());
  debug_pub_->publish(msg);
}

void BeamwalkingPolicy::init(
    const std::vector<double>& stance_kp, const std::vector<double>& stance_kd,
    const std::vector<double>& swing_kp, const std::vector<double>& swing_kd,
    const std::vector<double>& swing_kp_cart,
    const std::vector<double>& swing_kd_cart, const std::string& model_path,
    double policy_inference_rate,
    const std::vector<double>& stand_joint_angles) {
  // Base init loads the ONNX session, is_hardware, and the gains. It also
  // builds nominal_stance_pose_ from stand_joint_angles; this policy ignores
  // that and uses the Isaac per-joint default pose loaded below.
  LearnedVelocityPolicy::init(stance_kp, stance_kd, swing_kp, swing_kd,
                              swing_kp_cart, swing_kd_cart, model_path,
                              policy_inference_rate, stand_joint_angles);

  if (std::abs(policy_inference_rate_ - 1.0 / kControlDt) > 1e-6) {
    RCLCPP_WARN(node_->get_logger(),
                "BeamwalkingPolicy trained at %.0f Hz but "
                "policy_inference_rate is %.1f Hz; the phase clock assumes "
                "one tick per inference",
                1.0 / kControlDt, policy_inference_rate_);
  }

  loadGaitCommand();
  last_walk_cmd_time_ = node_->now();

  // Warm the ONNX session up now. The first Run() on the CUDA provider takes
  // several hundred ms (context + kernel setup), which used to stall the 500
  // Hz driver loop on the first walking tick, let cmd_vel go stale, and
  // trigger a walk stop / restart mid-stride. Pay that cost here instead.
  if (session_) {
    const auto t_start = std::chrono::steady_clock::now();
    obs_.setZero();
    obs_(8) = -1.0;  // level gravity, everything else zero
    for (int i = 0; i < 3; ++i) runInference();
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t_start)
                          .count();
    RCLCPP_INFO(node_->get_logger(),
                "BeamwalkingPolicy: ONNX warm-up done (%.1f ms for 3 runs)",
                ms);
  }
  resetWalk();
}

void BeamwalkingPolicy::loadGaitCommand() {
  std::string gait_name;
  std::vector<double> default_joint_pos;
  quad_utils::loadROSParam(node_, "robot_driver.beamwalking.speed", cmd_speed_);
  quad_utils::loadROSParam(node_, "robot_driver.beamwalking.duty_factor",
                           cmd_duty_);
  quad_utils::loadROSParam(node_, "robot_driver.beamwalking.step_width",
                           cmd_width_);
  quad_utils::loadROSParam(node_, "robot_driver.beamwalking.period",
                           cmd_period_);
  quad_utils::loadROSParam(node_, "robot_driver.beamwalking.gait", gait_name);
  quad_utils::loadROSParam(node_, "robot_driver.beamwalking.default_joint_pos",
                           default_joint_pos);
  quad_utils::loadROSParamDefault(node_, "robot_driver.beamwalking.action_scale",
                                  scale_factor_, 0.25);
  quad_utils::loadROSParamDefault(node_, "robot_driver.beamwalking.action_clip",
                                  action_clip_, 5.0);
  quad_utils::loadROSParamDefault(
      node_, "robot_driver.beamwalking.walk_cmd_threshold", walk_cmd_threshold_,
      0.05);
  quad_utils::loadROSParamDefault(node_,
                                  "robot_driver.beamwalking.hold_default_pose",
                                  hold_default_pose_, false);
  quad_utils::loadROSParamDefault(node_,
                                  "robot_driver.beamwalking.walk_stop_delay",
                                  walk_stop_delay_, 0.5);
  quad_utils::loadROSParamDefault(
      node_, "robot_driver.beamwalking.allow_narrow_width", allow_narrow_width_,
      false);
  quad_utils::loadROSParamDefault(
      node_, "robot_driver.beamwalking.lateral_heading_gain",
      lateral_heading_gain_, 0.0);
  quad_utils::loadROSParamDefault(node_,
                                  "robot_driver.beamwalking.use_fixed_course",
                                  use_fixed_course_, false);
  quad_utils::loadROSParamDefault(node_, "robot_driver.beamwalking.course_yaw",
                                  course_yaw_, 0.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.beamwalking.course_x",
                                  course_x_, 0.0);
  quad_utils::loadROSParamDefault(node_, "robot_driver.beamwalking.course_y",
                                  course_y_, 0.0);
  if (use_fixed_course_) {
    RCLCPP_INFO(node_->get_logger(),
                "BeamwalkingPolicy: fixed course, yaw %.3f rad through "
                "(%.2f, %.2f); heading and lateral offset are measured "
                "against it instead of the walk-start pose",
                course_yaw_, course_x_, course_y_);
  }
  if (hold_default_pose_) {
    RCLCPP_WARN(node_->get_logger(),
                "beamwalking.hold_default_pose is set: policy inference is "
                "disabled, holding the Isaac default pose at stance gains");
  }

  if (gait_name == "trot") {
    gait_id_ = 0;
  } else if (gait_name == "walk") {
    gait_id_ = 1;
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "beamwalking.gait must be 'trot' or 'walk', got '%s'; "
                 "using trot",
                 gait_name.c_str());
    gait_id_ = 0;
  }

  if (default_joint_pos.size() != 12) {
    RCLCPP_ERROR(node_->get_logger(),
                 "beamwalking.default_joint_pos needs 12 entries (Isaac joint "
                 "order), got %zu; falling back to the Isaac Go2 defaults",
                 default_joint_pos.size());
    default_joint_pos = {0.1, -0.1, 0.1, -0.1, 0.8, 0.8,
                         1.0, 1.0, -1.5, -1.5, -1.5, -1.5};
  }
  for (int i = 0; i < 12; ++i) default_joint_pos_(i) = default_joint_pos[i];

  // Clamp the scalar commands to the trained ranges; the policy only saw
  // normalized values in [-1, 1].
  auto clampWarn = [&](const char* name, double& v, const double range[2]) {
    if (v < range[0] || v > range[1]) {
      RCLCPP_WARN(node_->get_logger(),
                  "beamwalking.%s = %.3f outside trained range [%.3f, %.3f]; "
                  "clamping",
                  name, v, range[0], range[1]);
      v = std::clamp(v, range[0], range[1]);
    }
  };
  clampWarn("speed", cmd_speed_, kSpeedRange);
  clampWarn("duty_factor", cmd_duty_, kDutyRange);
  if (allow_narrow_width_ && cmd_width_ < kWidthRange[0] && cmd_width_ > 0.0) {
    RCLCPP_WARN(node_->get_logger(),
                "beamwalking.step_width = %.3f is below the trained minimum "
                "%.2f m; allow_narrow_width is set so the policy will "
                "extrapolate (normalized width %.2f)",
                cmd_width_, kWidthRange[0], normalizeCommand(cmd_width_, kWidthRange));
  } else {
    clampWarn("step_width", cmd_width_, kWidthRange);
  }

  // Period lives on the 50 Hz tick grid: protocol.PERIOD_TICKS is 18..27.
  period_ticks_ = static_cast<int>(std::lround(cmd_period_ / kControlDt));
  const int min_ticks =
      (gait_id_ == 1) ? kMinWalkPeriodTicks : kPeriodTicksRange[0];
  if (period_ticks_ < min_ticks || period_ticks_ > kPeriodTicksRange[1]) {
    RCLCPP_WARN(node_->get_logger(),
                "beamwalking.period = %.3f s (%d ticks) outside trained range "
                "[%d, %d] ticks for %s; clamping",
                cmd_period_, period_ticks_, min_ticks, kPeriodTicksRange[1],
                gait_name.c_str());
    period_ticks_ = std::clamp(period_ticks_, min_ticks, kPeriodTicksRange[1]);
  }
  cmd_period_ = period_ticks_ * kControlDt;

  // Walk was trained only at duty .75; every gait needs >= 5 swing ticks.
  if (gait_id_ == 1 && std::abs(cmd_duty_ - 0.75) > 1e-6) {
    RCLCPP_WARN(node_->get_logger(),
                "beamwalking: walk gait was trained only at duty_factor 0.75 "
                "(got %.3f); forcing 0.75",
                cmd_duty_);
    cmd_duty_ = 0.75;
  }
  const double max_duty = 1.0 - static_cast<double>(kMinSwingTicks) /
                                    static_cast<double>(period_ticks_);
  if (cmd_duty_ > max_duty) {
    RCLCPP_WARN(node_->get_logger(),
                "beamwalking: duty_factor %.3f leaves fewer than %d swing "
                "ticks at %d ticks/period; clamping to %.3f",
                cmd_duty_, kMinSwingTicks, period_ticks_, max_duty);
    cmd_duty_ = max_duty;
  }

  RCLCPP_INFO(node_->get_logger(),
              "BeamwalkingPolicy command: %s, speed %.2f m/s, duty %.3f, "
              "width %.2f m, period %.2f s (%d ticks), action scale %.2f, "
              "clip %.1f, lateral heading gain %.2f rad/m",
              gait_id_ == 1 ? "walk" : "trot", cmd_speed_, cmd_duty_,
              cmd_width_, cmd_period_, period_ticks_, scale_factor_,
              action_clip_, lateral_heading_gain_);
}

void BeamwalkingPolicy::resetWalk() {
  walking_ = false;
  phase_tick_ = 0;
  prev_clipped_action_.setZero();
  raw_actions_.setZero();
  actions_.setZero();
  first_inference_ = true;
}

void BeamwalkingPolicy::computeLegPhases(std::array<double, 4>& phase) const {
  // protocol.leg_phase: ((4*tick + quarters*period_ticks) % (4*period_ticks))
  //                     / (4*period_ticks)
  const int denom = 4 * period_ticks_;
  for (int leg = 0; leg < 4; ++leg) {
    const int num =
        (4 * phase_tick_ + kGaitQuarters[gait_id_][leg] * period_ticks_) %
        denom;
    phase[leg] = static_cast<double>(num) / static_cast<double>(denom);
  }
}

double BeamwalkingPolicy::warpPhase(double p) const {
  const double d = cmd_duty_;
  return (p < d) ? 0.5 * p / d : 0.5 + 0.5 * (p - d) / (1.0 - d);
}

void BeamwalkingPolicy::computeObservations(
    const quad_msgs::msg::RobotState& robot_state_msg) {
  const auto& q_raw = robot_state_msg.joints.position;
  const auto& qd_raw = robot_state_msg.joints.velocity;
  const bool joints_ok = q_raw.size() >= 12 && qd_raw.size() >= 12;
  if (!joints_ok) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "BeamwalkingPolicy: joint state has %zu/%zu entries",
                         q_raw.size(), qd_raw.size());
  }

  const auto& qm = robot_state_msg.body.pose.orientation;
  const Eigen::Quaterniond quat(qm.w, qm.x, qm.y, qm.z);

  int idx = 0;

  // 0-2 base_lin_vel: Isaac's root_lin_vel_b. Both the Gazebo estimator
  // plugin and the hardware comp filter publish body.twist.linear in the
  // WORLD frame, so rotate into the body frame here. On hardware without
  // mocap RobotDriver never fills twist.linear and this reads zero.
  {
    const auto& v = robot_state_msg.body.twist.linear;
    const Eigen::Vector3d v_b = quat.conjugate() * Eigen::Vector3d(v.x, v.y, v.z);
    obs_(idx++) = v_b.x();
    obs_(idx++) = v_b.y();
    obs_(idx++) = v_b.z();
  }

  // 3-5 base_ang_vel, body frame, unscaled. Hardware: IMU gyro. Sim: the
  // estimator plugin already stores the body-frame rate in twist.angular
  // (the cached IMU message is never filled on the sim path).
  {
    const auto& w = is_hardware_ ? last_imu_msg_.angular_velocity
                                 : robot_state_msg.body.twist.angular;
    obs_(idx++) = w.x;
    obs_(idx++) = w.y;
    obs_(idx++) = w.z;
  }

  // 6-8 projected_gravity = R_body_world * (0,0,-1)
  {
    const Eigen::Vector3d g_b = quat.conjugate() * Eigen::Vector3d(0, 0, -1);
    obs_(idx++) = g_b.x();
    obs_(idx++) = g_b.y();
    obs_(idx++) = g_b.z();
  }

  // 9-20 joint_pos_rel and 21-32 joint_vel_rel in Isaac order (grouped by
  // joint type, legs FL FR RL RR). Quad-SDK stores leg-major FL RL FR RR.
  for (int j = 0; j < 3; ++j) {
    for (int leg = 0; leg < 4; ++leg) {
      const int raw_idx = 3 * kQuadLegOfIsaac[leg] + j;
      const int isaac_idx = 4 * j + leg;
      obs_(9 + isaac_idx) =
          joints_ok ? q_raw[raw_idx] - default_joint_pos_(isaac_idx) : 0.0;
      obs_(21 + isaac_idx) = joints_ok ? qd_raw[raw_idx] : 0.0;
    }
  }
  idx = 33;

  // 33-44 previous clipped action (zero at walk start).
  for (int i = 0; i < 12; ++i) obs_(idx++) = prev_clipped_action_(i);

  // 45-52 sin / cos of the duty-warped phase, 59-62 desired contact from the
  // raw phase.
  std::array<double, 4> phase;
  computeLegPhases(phase);
  for (int leg = 0; leg < 4; ++leg) {
    obs_(45 + leg) = std::sin(2.0 * M_PI * warpPhase(phase[leg]));
    obs_(49 + leg) = std::cos(2.0 * M_PI * warpPhase(phase[leg]));
    obs_(59 + leg) = (phase[leg] < cmd_duty_) ? 1.0 : 0.0;
  }

  // 53-58 normalized command + gait one-hot.
  const double period_range[2] = {kPeriodTicksRange[0] * kControlDt,
                                  kPeriodTicksRange[1] * kControlDt};
  obs_(53) = normalizeCommand(cmd_speed_, kSpeedRange);
  obs_(54) = normalizeCommand(cmd_duty_, kDutyRange);
  obs_(55) = normalizeCommand(cmd_width_, kWidthRange);
  obs_(56) = normalizeCommand(cmd_period_, period_range);
  obs_(57) = (gait_id_ == 0) ? 1.0 : 0.0;
  obs_(58) = (gait_id_ == 1) ? 1.0 : 0.0;

  // 63 heading relative to the axis captured at walk start. Training resets
  // the robot within +-0.05 rad of its course axis and penalizes heading
  // error, so the policy steers back toward zero.
  {
    const double yaw = std::atan2(2.0 * (qm.w * qm.z + qm.x * qm.y),
                                  1.0 - 2.0 * (qm.y * qm.y + qm.z * qm.z));
    const auto& pos = robot_state_msg.body.pose.position;
    if (first_inference_) {
      if (use_fixed_course_) {
        heading_ref_ = course_yaw_;
        walk_origin_x_ = course_x_;
        walk_origin_y_ = course_y_;
      } else {
        heading_ref_ = yaw;
        walk_origin_x_ = pos.x;
        walk_origin_y_ = pos.y;
      }
    }
    double heading = wrapToPi(yaw - heading_ref_);
    if (lateral_heading_gain_ != 0.0) {
      // Signed distance left (+) of the straight line through the walk-start
      // position along the walk-start heading. Needs a world-frame position:
      // ground truth in sim, mocap-fused estimate on hardware.
      const double dx = pos.x - walk_origin_x_, dy = pos.y - walk_origin_y_;
      const double lateral =
          -std::sin(heading_ref_) * dx + std::cos(heading_ref_) * dy;
      heading += lateral_heading_gain_ * lateral;
    }
    obs_(63) = heading;
  }

  // 64-67 measured foot contact, Isaac leg order. The training env's contact
  // cache is cleared on reset, so the first observation of an episode sees
  // zeros; mirror that on the first inference of a walk.
  const auto& contact = last_foot_contact_msg_.contact_states;
  for (int leg = 0; leg < 4; ++leg) {
    const int quad_leg = kQuadLegOfIsaac[leg];
    const bool in_contact = !first_inference_ &&
                            static_cast<int>(contact.size()) > quad_leg &&
                            contact[quad_leg];
    obs_(64 + leg) = in_contact ? 1.0 : 0.0;
  }
}

void BeamwalkingPolicy::runInference() {
  if (!session_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "BeamwalkingPolicy: ONNX session not initialized");
    return;
  }

  if (input_name_.empty()) {
    Ort::AllocatorWithDefaultOptions alloc;
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      RCLCPP_ERROR(node_->get_logger(),
                   "BeamwalkingPolicy expects 1 input + 1 output; got %zu + %zu",
                   session_->GetInputCount(), session_->GetOutputCount());
      return;
    }
    input_name_ = session_->GetInputNameAllocated(0, alloc).get();
    output_name_ = session_->GetOutputNameAllocated(0, alloc).get();
    auto in_shape =
        session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() == 2 && in_shape[1] != kObsDim) {
      RCLCPP_ERROR(node_->get_logger(),
                   "BeamwalkingPolicy: model wants %lld inputs, controller "
                   "builds %d",
                   static_cast<long long>(in_shape[1]), kObsDim);
    }
  }

  Eigen::VectorXf obs_f = obs_.cast<float>();
  const int64_t input_shape[2] = {1, kObsDim};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info_, obs_f.data(), obs_f.size(), input_shape, 2);

  const char* in_names[] = {input_name_.c_str()};
  const char* out_names[] = {output_name_.c_str()};
  auto outs = session_->Run(Ort::RunOptions{nullptr}, in_names, &input_tensor,
                            1, out_names, 1);
  if (outs.size() != 1 || !outs[0].IsTensor()) {
    RCLCPP_ERROR(node_->get_logger(), "BeamwalkingPolicy: unexpected output");
    return;
  }
  const float* y = outs[0].GetTensorMutableData<float>();

  // rsl_rl clip_actions=5: the env only ever saw the clipped action, both as
  // the joint target and as the next step's last_action observation.
  Eigen::VectorXd clipped(kActionDim);
  for (int i = 0; i < kActionDim; ++i) {
    raw_actions_(i) = static_cast<double>(y[i]);
    clipped(i) = std::clamp(raw_actions_(i), -action_clip_, action_clip_);
  }
  prev_clipped_action_ = clipped;

  // q_target = default + scale * a, Isaac order -> Quad-SDK order
  // (FL: hip 0, thigh 4, calf 8; RL: 2, 6, 10; FR: 1, 5, 9; RR: 3, 7, 11).
  const Eigen::VectorXd target = default_joint_pos_ + scale_factor_ * clipped;
  for (int quad_leg = 0; quad_leg < 4; ++quad_leg) {
    // Inverse of kQuadLegOfIsaac: quad 0->FL(0), 1->RL(2), 2->FR(1), 3->RR(3).
    static constexpr int kIsaacLegOfQuad[4] = {0, 2, 1, 3};
    const int leg = kIsaacLegOfQuad[quad_leg];
    for (int j = 0; j < 3; ++j) {
      actions_(3 * quad_leg + j) = target(4 * j + leg);
    }
  }
}

bool BeamwalkingPolicy::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
  (void)grf_array_msg;
  const auto now = node_->now();

  if (hold_default_pose_) {
    // Static plant check: same pose and gains as Isaac's calibrate_stance().
    static constexpr int kIsaacLegOfQuad[4] = {0, 2, 1, 3};
    leg_command_array_msg.leg_commands.resize(num_feet_);
    for (int i = 0; i < num_feet_; ++i) {
      auto& leg = leg_command_array_msg.leg_commands.at(i);
      leg.motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        auto& cmd = leg.motor_commands.at(j);
        cmd.pos_setpoint = default_joint_pos_(4 * j + kIsaacLegOfQuad[i]);
        cmd.vel_setpoint = 0.0;
        cmd.torque_ff = 0.0;
        cmd.kp = stance_kp_.at(j);
        cmd.kd = stance_kd_.at(j);
      }
    }
    return true;
  }

  // Walk gate: cmd_vel is the heartbeat and enable. The policy has no
  // standing regime (trained speeds start at 0.25 m/s), so anything short of
  // a forward command hands control back to RobotDriver's stand pose. Once
  // walking, a short dropout is ridden through for walk_stop_delay_ rather
  // than stopping and restarting mid-stride.
  const bool cmd_fresh = (now - last_cmd_vel_msg_time_).seconds() < 0.1;
  const bool cmd_go = std::abs(cmd_vel_msg_(0)) > walk_cmd_threshold_;
  if (cmd_fresh && cmd_go) last_walk_cmd_time_ = now;
  if (!walking_) {
    if (!(cmd_fresh && cmd_go)) return false;
    walking_ = true;
    phase_tick_ = 0;
    prev_clipped_action_.setZero();
    first_inference_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "BeamwalkingPolicy: walk started, heading reference captured");
  } else if (!(cmd_fresh && cmd_go) &&
             (now - last_walk_cmd_time_).seconds() >= walk_stop_delay_) {
    RCLCPP_INFO(node_->get_logger(),
                "BeamwalkingPolicy: walk stopped (%s for %.2f s)",
                cmd_fresh ? "cmd_vel below threshold" : "cmd_vel stale",
                walk_stop_delay_);
    resetWalk();
    return false;
  }

  // One inference per control tick (50 Hz); PD tracks at the driver rate.
  // The driver loop runs at 500 Hz, so a plain ">= period since last run"
  // test fires on the first loop tick at or after 20 ms and averages ~21 ms,
  // which stretches the gait period by ~5%. Schedule against an accumulated
  // deadline instead so the mean rate is exactly policy_inference_rate_.
  const double period = 1.0 / policy_inference_rate_;
  if (first_inference_ || (now - next_inference_time_).seconds() >= 0.0) {
    computeObservations(robot_state_msg);
    runInference();
    publishDebug(robot_state_msg, phase_tick_);
    last_inference_time_ = now;
    if (first_inference_ ||
        (now - next_inference_time_).seconds() > 2.0 * period) {
      // Fresh start or we fell far behind: re-anchor rather than burst.
      next_inference_time_ = now + rclcpp::Duration::from_seconds(period);
    } else {
      next_inference_time_ += rclcpp::Duration::from_seconds(period);
    }
    first_inference_ = false;
    // The observation just consumed phase_tick_; advance for the next one
    // (protocol.advance_phase_ticks).
    phase_tick_ = (phase_tick_ + 1) % period_ticks_;
  }

  leg_command_array_msg.leg_commands.resize(num_feet_);
  for (int i = 0; i < num_feet_; ++i) {
    auto& leg = leg_command_array_msg.leg_commands.at(i);
    leg.motor_commands.resize(3);
    for (int j = 0; j < 3; ++j) {
      auto& cmd = leg.motor_commands.at(j);
      cmd.pos_setpoint = actions_(3 * i + j);
      cmd.vel_setpoint = 0.0;
      cmd.torque_ff = 0.0;
      cmd.kp = stance_kp_.at(j);
      cmd.kd = stance_kd_.at(j);
    }
  }
  return true;
}
