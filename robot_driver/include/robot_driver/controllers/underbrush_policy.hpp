#ifndef UNDERBRUSH_POLICY_H
#define UNDERBRUSH_POLICY_H

#include "robot_driver/controllers/learned_velocity_policy.hpp"
#include <quad_msgs/msg/foot_contact.hpp>
#include <array>

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
     per_leg_X  [10] = q_leg(3) + qd_leg(3) + foot_force(1) + tau_meas_leg(3)
     leg order = FL, FR, RL, RR (must match _LEG_NAMES in per_leg_gru_model.py)

   The action head is identical to the base policy's, so runInference() reuses
   LearnedVelocityPolicy::postProcessActions() (scale + nominal + Isaac->Quad
   reorder). Only scale_factor_ differs (0.5 for underbrush).
*/
class UnderbrushPolicy : public LearnedVelocityPolicy {
 public:
  UnderbrushPolicy(rclcpp::Node::SharedPtr node, const std::string& robot_ns,
                   std::shared_ptr<quad_utils::QuadKD2> quadKD);

  /**
   * @brief Zero out all per-leg GRU hidden states. Call on any mode
   *        transition (stand->walk, walk->stand, safety-recovery) — the
   *        hidden state carries regime-specific context that becomes
   *        stale on transitions.
   */
  void resetHiddenStates();

  /**
   * @brief Cache the latest per-foot contact reading (Unitree foot-force
   *        sensor). foot_force_raw is in Quad-SDK leg order (FL, RL, FR, RR)
   *        and feeds the per-leg foot_force observation.
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

 protected:
  // --- GRU shape constants (must match RslRlPerLegGRUModelCfg training) ---
  static constexpr int kGRUHidden = 32;
  static constexpr int kGRUNumLayers = 1;
  static constexpr int kBatch = 1;
  static constexpr int kPerLegObsDim = 10;
  static constexpr int kBodyObsDim = 21;
  static constexpr int kActionDim = 12;

  // --- Observation scales (must match VineWalkV28GRUObservationsCfg) ---
  /// base_ang_vel ObsTerm scale.
  static constexpr float kAngVelScale = 0.2f;
  /// q_leg scale. Sim uses absolute joint position (no scale). CONFIRM against
  /// the training config if the GRU used joint_pos_rel / a nonunity scale.
  static constexpr float kJointPosScale = 1.0f;
  /// qd_leg scale. Matches the base MLP policy's joint-velocity scale (0.05).
  /// CONFIRM against the training config.
  static constexpr float kJointVelScale = 0.05f;
  /// tau_meas ObsTerm scale. Unitree tau_est is already in N·m, so this is the
  /// only factor applied.
  static constexpr float kTauScale = 0.01f;
  /// foot_force ObsTerm scale, applied to a value in Newtons.
  static constexpr float kFootForceObsScale = 0.05f;
  /// CALIBRATION: converts the raw Unitree foot_force count into Newtons.
  /// The sim trained foot_force on PhysX contact-force magnitude (Newtons),
  /// but Unitree's LowState.foot_force is an uncalibrated int16 (~0–1000).
  /// Leaving this at 1.0 feeds raw counts × kFootForceObsScale, which is
  /// ~20–50× out of the trained distribution. Set to (Newtons per count) for
  /// your robot after a static-load calibration.
  static constexpr float kFootForceCountsToNewtons = 1.0f;

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
};

#endif  // UNDERBRUSH_POLICY_H
