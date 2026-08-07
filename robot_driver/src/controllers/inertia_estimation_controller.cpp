#include "robot_driver/controllers/inertia_estimation_controller.hpp"

// Which robot's knee envelope to emit. Flip manually when switching robots.
//   false  → Spirit40 original amplitudes (center 1.3, amp 1.3 in ctrl space)
//   true   → Go2-scaled (center 1.36, amp 0.8 in ctrl space; safe inside Go2
//            calf range [-2.72, -0.84] after (sign=1, offset=-pi) conversion)
static constexpr bool kUseGo2KneeEnvelope = true;

// Which single leg to excite. Set to 0..3 to flail only that leg (others
// hold their current observed pose); set to -1 to flail all four legs
// simultaneously (original Spirit40 behavior).
//   Convention on Go2 (from readBag verification):
//       0 = FR    1 = BR    2 = FL    3 = BL
static constexpr int kTargetLeg = 0;

InertiaEstimationController::InertiaEstimationController(
    rclcpp::Node::SharedPtr node, const std::string& robot_ns,
    std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LegController(node, robot_ns, quadKD) {}

bool InertiaEstimationController::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
  // Lazy-load the per-leg per-joint URDF-vs-controller convention coefficients
  // once. Same param paths (leg_<i>.joints.{abad|hip|knee}.{sign,offset}) as
  // quad_kd2.cpp uses in IK. Defaults preserve identity for Spirit40 whose
  // yaml doesn't populate them.
  if (!conv_loaded_) {
    joint_sign_.assign(num_feet_, std::vector<double>(3, 1.0));
    joint_offset_.assign(num_feet_, std::vector<double>(3, 0.0));
    const char* joint_key[3] = {"abad", "hip", "knee"};
    for (int i = 0; i < num_feet_; ++i) {
      const std::string p = "leg_" + std::to_string(i);
      for (int j = 0; j < 3; ++j) {
        quad_utils::loadROSParamDefault(
            node_, p + ".joints." + joint_key[j] + ".sign",
            joint_sign_[i][j], 1.0);
        quad_utils::loadROSParamDefault(
            node_, p + ".joints." + joint_key[j] + ".offset",
            joint_offset_[i][j], 0.0);
      }
    }
    conv_loaded_ = true;
  }

  if ((last_local_plan_msg_ == NULL) ||
      ((node_->now() - rclcpp::Time(last_local_plan_msg_->header.stamp))
           .seconds() >= 0.1)) {
    return false;
  } else {
    leg_command_array_msg.leg_commands.resize(num_feet_);

    // Define vectors for joint positions and velocities
    Eigen::VectorXd joint_positions(3 * num_feet_),
        joint_velocities(3 * num_feet_), foot_positions(3 * num_feet_),
        foot_velocities(3 * num_feet_), body_state(12);
    quad_utils::vectorToEigen(robot_state_msg.joints.position, joint_positions);
    quad_utils::vectorToEigen(robot_state_msg.joints.velocity,
                              joint_velocities);
    quad_utils::multiFootStateMsgToEigen(robot_state_msg.feet, foot_positions,
                                         foot_velocities);
    body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg.body);

    // Define vectors for state positions and velocities
    Eigen::VectorXd state_positions(3 * num_feet_ + 6),
        state_velocities(3 * num_feet_ + 6);
    state_positions << joint_positions, body_state.head(6);
    state_velocities << joint_velocities, body_state.tail(6);

    // Initialize variables for ff and fb
    Eigen::VectorXd tau_array(3 * num_feet_),
        tau_swing_leg_array(3 * num_feet_);

    // Get reference state and grf from local plan or traj + grf messages
    rclcpp::Time t_first_state(
        last_local_plan_msg_->states.front().header.stamp);
    double t_now =
        (node_->now() - rclcpp::Time(last_local_plan_msg_->state_timestamp))
            .seconds();  // Use time of state - RECOMMENDED
    // double t_now = (ros::Time::now() - last_local_plan_time_).toSec(); // Use
    // time of plan receipt double t_now = (ros::Time::now() -
    // t_first_state).toSec(); // Use time of first state in plan

    if ((t_now <
         (rclcpp::Time(last_local_plan_msg_->states.front().header.stamp) -
          t_first_state)
             .seconds()) ||
        (t_now >
         (rclcpp::Time(last_local_plan_msg_->states.back().header.stamp) -
          t_first_state)
             .seconds())) {
      // See inverse_dynamics_controller.cpp for context: must bail
      // here, otherwise the interpolation loop falls through with
      // unpopulated ref_state_msg_ and downstream feet[i] reads
      // segfault.
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "ID node couldn't find the correct ref state!");
      return false;
    }

    // Interpolate the local plan to get the reference state and ff GRF
    for (size_t i = 0; i < last_local_plan_msg_->states.size() - 1; i++) {
      if ((t_now >=
           (rclcpp::Time(last_local_plan_msg_->states[i].header.stamp) -
            t_first_state)
               .seconds()) &&
          (t_now <
           (rclcpp::Time(last_local_plan_msg_->states[i + 1].header.stamp) -
            t_first_state)
               .seconds())) {
        double t_interp =
            (t_now -
             (rclcpp::Time(last_local_plan_msg_->states[i].header.stamp) -
              t_first_state)
                 .seconds()) /
            (rclcpp::Time(last_local_plan_msg_->states[i + 1].header.stamp)
                 .seconds() -
             rclcpp::Time(last_local_plan_msg_->states[i].header.stamp)
                 .seconds());

        // Linearly interpolate between states
        quad_utils::interpRobotState(last_local_plan_msg_->states[i],
                                     last_local_plan_msg_->states[i + 1],
                                     t_interp, ref_state_msg_);

        // ZOH on GRFs
        grf_array_msg = last_local_plan_msg_->grfs[i];

        break;
      }
    }

    // Declare plan and state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), grf_array(3 * num_feet_),
        ref_foot_positions(3 * num_feet_), ref_foot_velocities(3 * num_feet_),
        ref_foot_acceleration(3 * num_feet_);

    // Load plan and state data from messages
    quad_utils::multiFootStateMsgToEigen(
        ref_state_msg_.feet, ref_foot_positions, ref_foot_velocities,
        ref_foot_acceleration);
    grf_array = quad_utils::grfArrayMsgToEigen(grf_array_msg);
    if (last_grf_array_.norm() >= 1e-3) {
      grf_array = grf_exp_filter_const_ * grf_array.array() +
                  (1 - grf_exp_filter_const_) * last_grf_array_.array();
      quad_utils::eigenToGRFArrayMsg(grf_array, ref_state_msg_.feet,
                                     grf_array_msg);
    }

    // Load contact mode
    std::vector<int> contact_mode(num_feet_);
    for (int i = 0; i < num_feet_; i++) {
      contact_mode[i] = ref_state_msg_.feet.feet[i].contact;
    }

    // Compute joint torques
    quadKD_->computeInverseDynamics(ref_foot_acceleration, grf_array,
                                    contact_mode, tau_array);

    // Convert gains to eigen vectors for easier math
    Eigen::VectorXd swing_kp_cart_eig, swing_kd_cart_eig,
        swing_cart_fb(3 * num_feet_);
    quad_utils::vectorToEigen(swing_kp_cart_, swing_kp_cart_eig);
    quad_utils::vectorToEigen(swing_kd_cart_, swing_kd_cart_eig);

    // Compute PD feedback in cartesian space
    Eigen::MatrixXd jacobian(3 * num_feet_, state_positions.size());
    swing_cart_fb = swing_kp_cart_eig.replicate<4, 1>().cwiseProduct(
                        ref_foot_positions - foot_positions) +
                    swing_kd_cart_eig.replicate<4, 1>().cwiseProduct(
                        ref_foot_velocities - foot_velocities);

    // Transform PD into joint space
    quadKD_->getJacobianBodyAngVel(jacobian);
    swing_cart_fb =
        jacobian.block(0, 0, 3 * num_feet_, 3 * num_feet_).transpose() *
        swing_cart_fb;

    double t = node_->now().seconds();

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);

      // Single-leg gating: if kTargetLeg is set (0..3) and this isn't it,
      // hold this leg at its currently observed pose (wire convention — no
      // sign/offset conversion needed since it's already in wire space).
      // kTargetLeg = -1 flails all legs simultaneously.
      if (kTargetLeg >= 0 && i != kTargetLeg) {
        for (int j = 0; j < 3; ++j) {
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(j)
              .pos_setpoint = robot_state_msg.joints.position.at(3 * i + j);
        }
      } else {

      // ---- Frequencies halved from Spirit40 original ----
      // Motivation: original envelope had instantaneous frequencies up to
      // ~25 rad/s (via cos(25*sin(1.4t)) etc). At Go2's reflected calf
      // inertia (~3.4 kg m²) tracking those needed ~640 N m — 18× the
      // 35.55 N m knee limit. Physical bandwidth limit, unfixable by kp.
      // Halving every frequency puts peak instantaneous content near
      // 2 Hz, which Go2 can track cleanly.
      //
      // ---- Smooth gate blending (replaces Spirit40's hard step) ----
      // Original used `(sin(gate) < 0.8)` as a 1.0/0.0 step, causing
      // 0.1-0.3 rad discontinuous jumps in pos_setpoint every ~0.4 s at
      // each gate crossing → instantaneous ~12 N m torque demand → jerk.
      // Replace with a tanh sigmoid so w_main+w_rest is C^inf-continuous.
      // Sharpness = 6 gives a ~0.05 s transition window.
      auto w_main = [](double s) {
        return 0.5 + 0.5 * std::tanh(6.0 * (0.8 - s));
      };

      // Abad envelope — reduced amplitude (0.25x Spirit40 original).
      // Stand hardware sits directly under the body, so abad swinging the
      // leg inward can bump into it. Keep total abad excursion under
      // ~0.15 rad from stand-nominal.
      {
        const double wa = w_main(sin(2.7 * t));
        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(0)
            .pos_setpoint =
            (wa * (0.05 * sin(2 * t) + 0.1 * cos(7.5 * sin(1.3 * t))) +
             (1.0 - wa) * (0.12 * sin(0.35 * t))) *
            joint_sign_[i][0] + joint_offset_[i][0];
      }

      // Hip pitch — 0.7x Spirit40 amplitude with halved frequencies.
      // Ctrl range [-0.75, +1.35] → Go2 wire [+0.22, +2.32] via
      // (sign=-1, offset=pi/2). Vertical plane is unobstructed by stand.
      {
        const double wh = w_main(sin(3.25 * t));
        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(1)
            .pos_setpoint =
            (wh * (-0.7 * cos(3 * t) + 0.3 - 0.35 * cos(10 * sin(1.1 * t))) +
             (1.0 - wh) * (0.7 * sin(0.45 * t) + 0.5)) *
            joint_sign_[i][1] + joint_offset_[i][1];
      }

      // Knee — same amplitude structure as before, frequencies halved.
      // Go2 branch: ctrl range [+0.56, +2.16] → wire [-2.58, -0.98], safely
      // inside Go2 calf range [-2.72, -0.84].
      double q_ctrl_knee;
      const double wk = w_main(sin(3.5 * t));
      if (kUseGo2KneeEnvelope) {
        q_ctrl_knee =
            wk * (-0.492 * cos(5 * t) + 1.36 -
                  0.308 * cos(12.5 * sin(0.7 * t))) +
            (1.0 - wk) * (0.406 * sin(0.55 * t) + 1.36);
      } else {
        q_ctrl_knee =
            wk * (-0.8 * cos(5 * t) + 1.3 -
                  0.5 * cos(12.5 * sin(0.7 * t))) +
            (1.0 - wk) * (0.7 * sin(0.55 * t) + 0.7);
      }
      leg_command_array_msg.leg_commands.at(i)
          .motor_commands.at(2)
          .pos_setpoint = q_ctrl_knee * joint_sign_[i][2] + joint_offset_[i][2];

      }  // end excitation branch

      // Dedicated flail PD gains — INDEPENDENT of swing_kp_/swing_kd_ used
      // for underbrush walking (which are ~10/1, too soft to track ±1 rad
      // pos_setpoint references against gravity + leg inertia). Chosen so
      // max restoring torque (kp × max_ref_amplitude) stays within motor
      // limits: hip/abad limit 23.7 Nm, knee 45.4 Nm. Bump these if
      // tracking is still lagging in the recorded bags.
      //   abad: kp*amp ≈ 20*0.15 = 3 Nm  (well within limit; amp small
      //         because of stand collision constraint)
      //   hip:  kp*amp ≈ 20*1.05 = 21 Nm (near hip motor limit 23.7)
      //   knee: kp*amp ≈ 30*0.80 = 24 Nm (well within knee limit 45.4)
      constexpr double flail_kp[3] = {20.0, 20.0, 30.0};  // abad, hip, knee
      constexpr double flail_kd[3] = {2.0,  2.0,  3.0};
      for (int j = 0; j < 3; ++j) {
        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(j)
            .vel_setpoint = 0;
        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(j)
            .torque_ff = 0;
        // For the non-flailed legs (kTargetLeg gating), use SOFT gains
        // (swing_kp_/kd_) so they hold their observed pose gently; for the
        // flailed leg, use the stiff flail gains above.
        const bool is_flail_leg = (kTargetLeg < 0) || (i == kTargetLeg);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
            is_flail_leg ? flail_kp[j] : swing_kp_.at(j);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
            is_flail_leg ? flail_kd[j] : swing_kd_.at(j);
      }
    }

    last_grf_array_ = grf_array;
    return true;
  }
}
