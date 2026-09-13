#ifndef UNDERBRUSH_POLICY_H
#define UNDERBRUSH_POLICY_H

#include "robot_driver/controllers/learned_velocity_policy.hpp"
#include <quad_msgs/msg/foot_contact.hpp>
#include <array>
#include <std_msgs/msg/float32_multi_array.hpp>

//! Recurrent per-leg-GRU + MLP policy trained on the underbrush vine field.
/*!
   UnderbrushPolicy is a specialization of LearnedVelocityPolicy that handles
   the underbrush per-leg-GRU actor architecture. It differs from a plain MLP
   policy in three ways:

     1. Multiple ONNX inputs: 4 per-leg observation tensors (10 dims each),
        1 body observation tensor (21 dims), and 4 GRU hidden state tensors
        (one per leg, shape [num_layers, batch=1, hidden]).
     2. Multiple ONNX outputs: 1 action tensor (12 dims) + 4 updated hidden
        state tensors that must be fed back on the next inference.
     3. Persistent state: hidden states must be preserved between step()
        calls and zeroed on mode transitions (call resetHiddenStates()).

   Observation layout (must match VineWalkV28GRUObservationsCfg):
     body       [21] = base_ang_vel(3, scale=0.2) + projected_gravity(3) +
                       velocity_commands(3) + last_action(12)
     per_leg_X  [10] = q_leg(3) + qd_leg(3) + joint_effort(3) + foot_contact(1)
     leg order = FL, FR, RL, RR (must match _LEG_NAMES in per_leg_gru_model.py)

   The action head is identical to the base policy's, so runInference() reuses
   LearnedVelocityPolicy::postProcessActions() (scale + nominal + Isaac->Quad
   reorder). Only scale_factor_ differs (0.5 for underbrush).
*/
class UnderbrushPolicy : public LearnedVelocityPolicy {
 public:
  UnderbrushPolicy(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                   std::shared_ptr<quad_utils::QuadKD2> quadKD,
                   bool per_leg_action_history = false);

  /**
   * @brief Zero out all per-leg GRU hidden states AND arm the encoder-warmup
   *        counter. Call on any mode transition (stand->walk, walk->stand,
   *        safety-recovery) — the hidden state carries regime-specific
   *        context that becomes stale on transitions.
   *
   *        Deploy-fix: v51 was never trained on the (h=0 ∧ settled) state
   *        that occurs at Gazebo/hardware handoff, so raw MLP output rails at
   *        tick 0. After reset, we run ONNX forward as normal (to fill hidden
   *        with contextually-driven state) but hold PD-nominal joint targets
   *        for kEncoderWarmupTicks before releasing MLP output. See docs and
   *        the reset_condition videos.
   */
  void resetHiddenStates();

  /**
   * @brief Cache the latest per-foot contact reading (Unitree foot-force
   *        sensor). The binary contact_states (Quad-SDK leg order FL, RL, FR,
   *        RR) feed the per-leg foot_force observation.
   */
  void updateFootContactMsg(const quad_msgs::msg::FootContact& msg);

  /**
   * @brief Split the flat robot_state into per-leg + body observation
   *        groups matching the sim's observation manager.
   */
  void computeObservations(
      const quad_msgs::msg::RobotState& robot_state_msg) override;

  /**
   * @brief Build 9 input tensors (4 per-leg + body + 4 hidden), run the
   *        session, copy the 4 updated hidden states back into member
   *        storage, and post-process actions into Quad-SDK joint targets.
   */
  void runInference() override;

  /**
   * @brief Override init so we can restore our v51-specific gains + asymmetric
   *        nominal stance AFTER the parent's init overwrites them from yaml.
   *        Parent init reads stance_kp/kd + stand_joint_angles from robot_driver
   *        config, but v51 was trained with kp=25/kd=0.5 and asymmetric per-leg
   *        defaults (Isaac's `.*L_hip = +0.1, .*R_hip = -0.1, F/R thigh differ`).
   */
  void init(const std::vector<double>& stance_kp,
            const std::vector<double>& stance_kd,
            const std::vector<double>& swing_kp,
            const std::vector<double>& swing_kd,
            const std::vector<double>& swing_kp_cart,
            const std::vector<double>& swing_kd_cart,
            const std::string& model_path,
            double policy_inference_rate,
            const std::vector<double>& stand_joint_angles) override;

 protected:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_debug_pub_;
  // --- GRU shape constants (must match v51's actual trained weights) ---
  // NB: RslRlPerLegGRUModelCfg default is 256 but PerLegGRUModel default is 64,
  // and v51's actual weights are 64 (verified from ckpt weight_ih_l0 shape
  // (192, 10) = 3*64). The runner cfg's 256 was never actually plumbed through.
  static constexpr int kGRUHidden = 64;
  static constexpr int kGRUNumLayers = 1;
  static constexpr int kBatch = 1;
  static constexpr int kPerLegObsDim = 10;
  const int per_leg_obs_dim_;  // V90: 10; V92: 13 (append previous raw actions).
  static constexpr int kBodyObsDim = 21;
  static constexpr int kActionDim = 12;

  // --- Observation scales (must match v51's vinewalk_gru obs config) ---
  /// base_ang_vel ObsTerm scale.
  static constexpr float kAngVelScale = 0.2f;
  /// q_leg is joint_pos_rel (q - default). Scale 1.0; the nominal subtraction
  /// is done in computeObservations, mirroring LearnedVelocityPolicy.
  static constexpr float kJointPosScale = 1.0f;
  /// qd_leg (joint_vel_rel; default vel is 0 so ≈ absolute) ObsTerm scale.
  static constexpr float kJointVelScale = 0.05f;
  /// Simulator/Unitree torque observation scale, matching training.
  static constexpr float kTauScale = 0.01f;
  // foot_force is a BINARY contact flag in v51 (sim force > 5 N, scale 1.0);
  // it is fed straight from the interface's contact_states — no scale constant.

  /// Isaac leg index (0=FL,1=FR,2=RL,3=RR) → Quad-SDK leg index
  /// (0=FL,1=RL,2=FR,3=RR). Used to pull the right joints / foot force out of
  /// the Quad-SDK-ordered robot_state.
  static constexpr int kQuadLegOfIsaac[4] = {0, 2, 1, 3};

  /// Per-leg observation buffers (contiguous float storage for ONNX)
  std::array<std::vector<float>, 4> obs_per_leg_;

  /// Body observation buffer
  std::vector<float> obs_body_;

  /// Per-leg GRU hidden states.
  /// Layout: [num_layers, batch, hidden_size] flattened.
  std::array<std::vector<float>, 4> h_state_;

  /// Latest cached foot-contact reading (Quad-SDK leg order).
  quad_msgs::msg::FootContact last_foot_contact_msg_;

  /// Cached ONNX I/O metadata (populated lazily on first runInference)
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<const char*> in_name_cstrs_;
  std::vector<const char*> out_name_cstrs_;

  /// Leg index → name mapping. 0=FL, 1=FR, 2=RL, 3=RR.
  static constexpr const char* kLegNames[4] = {"FL", "FR", "RL", "RR"};

  /// Encoder-warmup: number of policy ticks after any hidden-state reset
  /// during which we run the ONNX encoder forward (to fill hidden with real
  /// obs-driven context) but override the MLP head output to zero (→ actions
  /// = nominal stance via postProcessActions). Rationale: v51 was trained on
  /// (h!=0) settled states but never (h=0 ∧ settled), so the raw MLP output
  /// rails at deploy handoff. Empirically the hidden fills within ~30 ticks
  /// (~600ms at 50Hz) — long enough to escape the OOD region, short enough to
  /// not delay operator commands appreciably.
  static constexpr int kEncoderWarmupTicks = 30;

  /// Warmup counter (decremented in runInference; 0 → normal closed-loop).
  int warmup_ticks_remaining_ = 0;
};

#endif  // UNDERBRUSH_POLICY_H
