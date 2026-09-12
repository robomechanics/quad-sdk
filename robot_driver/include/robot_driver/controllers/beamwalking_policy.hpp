#ifndef BEAMWALKING_POLICY_H
#define BEAMWALKING_POLICY_H

#include "robot_driver/controllers/learned_velocity_policy.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <string>
#include <vector>

//! Gait-command MLP policy trained for narrow-support (beam) walking.
/*!
   BeamwalkingPolicy is a specialization of LearnedVelocityPolicy for the
   68-input / 12-output flat-ground gait-command actor exported from the
   beam_walking_RL Isaac Lab task (experiment/task.py + protocol.py). It
   differs from the base velocity policy in three ways:

     1. The command is a gait specification (forward speed, duty factor, step
        width, period, gait type) loaded from the robot yaml, not a twist.
        cmd_vel is only the walk enable / heartbeat.
     2. The policy carries an explicit 50 Hz integer phase clock. The clock is
        zeroed on walk start and advanced once per inference, exactly like the
        training environment's command term.
     3. Observations are unscaled (no 0.2 / 0.05 factors) and relative to the
        Isaac Go2 per-joint default pose, not the uniform stand pose.

   Observation layout (zero-based, must match the export README and task.py):
      0- 2  base linear velocity, body frame              [m/s]
      3- 5  base angular velocity, body frame             [rad/s]
      6- 8  unit gravity in body frame, level = (0,0,-1)
      9-20  joint position - default, Isaac joint order   [rad]
     21-32  joint velocity, Isaac joint order             [rad/s]
     33-44  previous clipped action, Isaac joint order
     45-48  sin(2*pi*warped phase), FL FR RL RR
     49-52  cos(2*pi*warped phase), FL FR RL RR
     53-56  normalized speed, duty, width, period in [-1,1]
     57-58  gait one-hot: trot [1,0], walk [0,1]
     59-62  desired contact = raw phase < duty, FL FR RL RR
     63     yaw relative to the heading captured at walk start [rad]
     64-67  measured foot contact, FL FR RL RR

   Isaac joint order is grouped by joint type:
     FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, ..., FL_calf, ..., RR_calf.

   Action head: q_target = default_joint_pos + action_scale * clip(a, +-clip).
   The clipped action is what the next observation sees as "previous action".
*/
class BeamwalkingPolicy : public LearnedVelocityPolicy {
 public:
  BeamwalkingPolicy(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                    std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Load gains, the ONNX model, and the gait command block from the
   *        parameter server. stand_joint_angles is accepted for interface
   *        compatibility but not used: the policy's default pose is the
   *        Isaac per-joint pose from beamwalking.default_joint_pos.
   */
  void init(const std::vector<double>& stance_kp,
            const std::vector<double>& stance_kd,
            const std::vector<double>& swing_kp,
            const std::vector<double>& swing_kd,
            const std::vector<double>& swing_kp_cart,
            const std::vector<double>& swing_kd_cart,
            const std::string& model_path, double policy_inference_rate = 50.0,
            const std::vector<double>& stand_joint_angles = {0.0, 0.8,
                                                              -1.5}) override;

  /**
   * @brief Assemble the 68-dim observation from the robot state, the cached
   *        IMU / foot-contact messages, the phase clock, and the gait command.
   */
  void computeObservations(
      const quad_msgs::msg::RobotState& robot_state_msg) override;

  /**
   * @brief Run the MLP, clip the raw action, and convert to joint targets.
   */
  void runInference() override;

  /**
   * @brief Walk gating + inference cadence + PD tracking. Returns false (so
   *        RobotDriver holds the stand pose) while cmd_vel is stale or the
   *        forward command is below the walk threshold. On the first tick of
   *        a walk it captures the heading reference and zeroes the phase clock
   *        and previous action; after each inference it advances the clock.
   */
  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState& robot_state_msg,
      quad_msgs::msg::LegCommandArray& leg_command_array_msg,
      quad_msgs::msg::GRFArray& grf_array_msg) override;

  /**
   * @brief Return to the "not walking" state: next walk start re-captures the
   *        heading reference and restarts the phase clock from tick 0.
   */
  void resetWalk();

 protected:
  /// Read and validate the beamwalking.* parameter block.
  void loadGaitCommand();

  /// Raw leg phase in [0,1) for the current tick, Isaac leg order. Integer
  /// arithmetic mirrors protocol.leg_phase so quarter-cycle offsets that fall
  /// between ticks round identically to training.
  void computeLegPhases(std::array<double, 4>& phase) const;

  /// Duty-warped phase (protocol.duty_warped_phase): commanded liftoff lands
  /// at encoded phase 0.5 for every duty factor.
  double warpPhase(double phase) const;

  // --- Interface constants (export README / protocol.py) ---
  static constexpr int kObsDim = 68;
  static constexpr int kActionDim = 12;
  static constexpr double kControlDt = 0.02;  // protocol.CONTROL_DT
  static constexpr double kSpeedRange[2] = {0.25, 0.40};
  static constexpr double kDutyRange[2] = {0.50, 0.75};
  static constexpr double kWidthRange[2] = {0.10, 0.50};
  static constexpr int kPeriodTicksRange[2] = {18, 27};
  static constexpr int kMinSwingTicks = 5;      // protocol.MIN_SWING_STEPS
  static constexpr int kMinWalkPeriodTicks = 20;
  /// Leg phase offsets in quarter cycles, Isaac leg order FL, FR, RL, RR.
  /// protocol.GAIT_OFFSETS: trot (0, .5, .5, 0), walk (0, .75, .5, .25).
  static constexpr int kGaitQuarters[2][4] = {{0, 2, 2, 0}, {0, 3, 2, 1}};

  /// Isaac leg index (0=FL,1=FR,2=RL,3=RR) -> Quad-SDK leg index
  /// (0=FL,1=RL,2=FR,3=RR).
  static constexpr int kQuadLegOfIsaac[4] = {0, 2, 1, 3};

  // --- Gait command (beamwalking.* params) ---
  double cmd_speed_ = 0.30;       // m/s
  double cmd_duty_ = 0.625;       // stance fraction
  double cmd_width_ = 0.30;       // full stance width, m
  double cmd_period_ = 0.48;      // s
  int gait_id_ = 0;               // 0 = trot, 1 = walk
  int period_ticks_ = 24;         // round(period / 0.02)
  double action_clip_ = 5.0;
  /// |cmd_vel.linear.x| above this starts / keeps the walk.
  double walk_cmd_threshold_ = 0.05;
  /// Grace period before a stale or dropped cmd_vel ends the walk. A restart
  /// mid-stride (phase clock and previous action reset, stand gains for a
  /// tick) is far more destabilizing than continuing a few steps, and the
  /// driver's own input/heartbeat timeouts remain the hard safety stop.
  double walk_stop_delay_ = 0.5;
  /// Time the walk gate last saw a fresh, above-threshold command.
  rclcpp::Time last_walk_cmd_time_;
  /// Let step_width go below the trained 0.10 m (extrapolation; the network
  /// then sees a normalized width < -1). Off by default: clamps instead.
  bool allow_narrow_width_ = false;
  /// Lateral-drift correction through the heading input the policy already
  /// has: heading_obs += gain * (lateral offset from the walk-start line). The
  /// policy steers heading back to zero, so a positive offset (left of the
  /// line) makes it turn right. rad per metre; 0 disables.
  double lateral_heading_gain_ = 0.0;
  /// Walk-start pose that defines the straight-line frame for the lateral
  /// offset: world position and the heading captured at walk start.
  double walk_origin_x_ = 0.0, walk_origin_y_ = 0.0;
  /// Fixed course instead of the walk-start line: when use_fixed_course is
  /// set, the heading reference is course_yaw and the centreline passes
  /// through (course_x, course_y) in the world frame. Use this on a beam so
  /// a slightly yawed start does not define a line that leaves the beam.
  bool use_fixed_course_ = false;
  double course_yaw_ = 0.0, course_x_ = 0.0, course_y_ = 0.0;
  /// Diagnostic: skip inference and hold default_joint_pos at the stance
  /// gains. Lets the settled height / joint sag be compared against Isaac's
  /// stance calibration (settled_stance.json) with the same Kp/Kd.
  bool hold_default_pose_ = false;

  /// Isaac per-joint default pose, Isaac joint order. Replaces the base
  /// class's nominal_stance_pose_ for both observations and the action head.
  Eigen::VectorXd default_joint_pos_{Eigen::VectorXd::Zero(12)};

  // --- Walk state ---
  bool walking_ = false;
  int phase_tick_ = 0;
  /// Accumulated deadline for the next inference (see computeLegCommandArray).
  rclcpp::Time next_inference_time_;
  double heading_ref_ = 0.0;
  /// Clipped action from the previous inference (obs 33-44).
  Eigen::VectorXd prev_clipped_action_{Eigen::VectorXd::Zero(12)};

  /// Cached ONNX I/O names (populated on first inference).
  std::string input_name_, output_name_;

  /// Debug stream, one message per inference, on beamwalking/policy_debug:
  ///   [0..67]  observation exactly as fed to the network
  ///   [68..79] raw network action, Isaac joint order
  ///   [80..91] clipped action (what the next obs sees as previous action)
  ///   [92]     phase tick consumed by this observation
  ///   [93]     stamp of the robot state used, seconds
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  void publishDebug(const quad_msgs::msg::RobotState& robot_state_msg,
                    int tick) const;
};

#endif  // BEAMWALKING_POLICY_H
