#include "robot_driver/controllers/underbrush_inverse_dynamics.hpp"

UnderbrushInverseDynamicsController::UnderbrushInverseDynamicsController(
    rclcpp::Node::SharedPtr node, const std::string& robot_ns,
    std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LegController(node, robot_ns, quadKD) {
  force_mode_ = {0, 0, 0, 0};
  last_mode_ = {0, 0, 0, 0};

  double t_now = node_->now().seconds();
  t_switch_ = {t_now, t_now, t_now, t_now};
  t_LO_ = {t_now, t_now, t_now, t_now};
  t_TD_ = {t_now, t_now, t_now, t_now};
}

void UnderbrushInverseDynamicsController::updateBodyForceEstimate(
    const quad_msgs::msg::BodyForceEstimate::SharedPtr msg) {
  last_body_force_estimate_msg_ = msg;
}

void UnderbrushInverseDynamicsController::setUnderbrushParams(
    double retract_vel, double tau_push, double tau_contact_start,
    double tau_contact_end, double min_switch, double t_down, double t_up,
    double hip_retract_sign) {
  retract_vel_ = retract_vel;
  tau_push_ = tau_push;
  tau_contact_start_ = tau_contact_start;
  tau_contact_end_ = tau_contact_end;
  min_switch_ = min_switch;
  t_down_ = t_down;
  t_up_ = t_up;
  hip_retract_sign_ = hip_retract_sign;
}

bool UnderbrushInverseDynamicsController::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
  if ((last_local_plan_msg_ == NULL || last_body_force_estimate_msg_ == NULL) ||
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
    quad_msgs::msg::RobotState ref_underbrush_msg, ref_abad_msg;
    Eigen::VectorXd tau_array(3 * num_feet_),
        tau_swing_leg_array(3 * num_feet_);

    // Get reference state and grf from local plan or traj + grf messages
    rclcpp::Time t_first_state(
        last_local_plan_msg_->states.front().header.stamp);
    double t_now =
        (node_->now() - rclcpp::Time(last_local_plan_msg_->state_timestamp))
            .seconds();  // Use time of state - RECOMMENDED
    double t_now2 = node_->now().seconds();

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
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "ID node couldn't find the correct ref state!");
      return false;
    }

    int all_TD = 0;  // end looping when all next touchdowns have been found

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

        // ref_state_msg = last_local_plan_msg_->states[i];
        grf_array_msg = last_local_plan_msg_->grfs[i];

        // Don't switch immediately after beginning swing
        for (int j = 0; j < num_feet_; j++) {
          if (last_local_plan_msg_->states[i].feet.feet.at(j).contact &&
              !last_local_plan_msg_->states[i + 1].feet.feet.at(j).contact) {
            // t_switch_.at(j) =
            // last_local_plan_msg_->states[i+1].header.stamp.toSec();
            t_LO_.at(j) =
                rclcpp::Time(last_local_plan_msg_->states[i + 1].header.stamp)
                    .seconds();
          }
        }

        // break;

      } else if (t_now <
                 rclcpp::Time(last_local_plan_msg_->states[i + 1].header.stamp)
                     .seconds()) {  // find next touchdowns
        if (all_TD == pow(2.0, num_feet_) -
                          1) {  // terminate when all next touchdowns are found
          break;
        }
        for (int j = 0; j < num_feet_; j++) {
          if (!(all_TD & (1 << j)) &&
              i > 0) {  // touchdown not yet found for this foot
            if (ref_state_msg_.feet.feet.at(j).contact) {
              all_TD =
                  all_TD |
                  (1 << j);  // foot is in stance, don't need next touchdown
            } else {
              if (last_local_plan_msg_->states[i].feet.feet.at(j).contact) {
                t_TD_.at(j) =
                    rclcpp::Time(last_local_plan_msg_->states[i].header.stamp)
                        .seconds();
                all_TD = all_TD | (1 << j);  // next touchdown found
              }
            }
          }
        }
      }
    }

    double foot_x_err, foot_y_err, foot_z_err, foot_horz_err, foot_z_hip;
    double foot_vx, foot_vy, foot_vz;

    // Underbrush swing leg motion
    ref_underbrush_msg = ref_state_msg_;
    for (int i = 0; i < 4; i++) {
      if (!ref_state_msg_.feet.feet.at(i).contact) {
        for (int j = 0; j < last_local_plan_msg_->states.size() - 1; j++) {
          if (t_now < rclcpp::Time(last_local_plan_msg_->states[j].header.stamp)
                          .seconds() &&
              bool(last_local_plan_msg_->states[j].feet.feet.at(i).contact)) {
            ref_underbrush_msg.feet.feet.at(i).position.x =
                robot_state_msg.feet.feet.at(i).position.x;
            ref_underbrush_msg.feet.feet.at(i).position.y =
                robot_state_msg.feet.feet.at(i).position.y;
            ref_underbrush_msg.feet.feet.at(i).position.z =
                robot_state_msg.feet.feet.at(i).position.z;

            // foot cartesian distances from desired footfall
            foot_x_err =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.x -
                robot_state_msg.feet.feet.at(i).position.x;
            foot_y_err =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.y -
                robot_state_msg.feet.feet.at(i).position.y;
            foot_z_err =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.z -
                robot_state_msg.feet.feet.at(i).position.z;
            foot_z_hip = robot_state_msg.feet.feet.at(i).position.z -
                         robot_state_msg.body.pose.position.z;
            foot_horz_err = foot_x_err;

            // cartesian foot velocity commands
            foot_vx = 10.0 * foot_x_err;
            foot_vx =
                abs(foot_vx) > 4.0 ? (foot_vx > 0 ? 1 : -1) * 4.0 : foot_vx;
            foot_vy = 10.0 * foot_y_err;
            foot_vy =
                abs(foot_vy) > 4.0 ? (foot_vy > 0 ? 1 : -1) * 4.0 : foot_vy;
            foot_vz = 10.0 * foot_z_err;
            if (foot_horz_err > 0.02 && t_TD_.at(i) - t_now2 > t_down_) {
              // don't put the foot down unless it's close to the right x, y
              // position or there's no time left
              foot_vz = 1.0 / (100 * (foot_horz_err - 0.02) + 1) * foot_vz;
            }
            if (foot_z_hip > -0.05) {
              // foot is too high above hip; singularity problems
              foot_vx = 1 / (50 * (foot_z_hip + 0.05) + 1) * foot_vx;
              foot_vy = 1 / (50 * (foot_z_hip + 0.05) + 1) * foot_vy;
              foot_vz += -10.0 * (foot_z_hip + 0.05);
            }
            foot_vz =
                abs(foot_vz) > 4.0 ? (foot_vz > 0 ? 1 : -1) * 4.0 : foot_vz;

            ref_underbrush_msg.feet.feet.at(i).velocity.x = foot_vx;
            ref_underbrush_msg.feet.feet.at(i).velocity.y = foot_vy;
            ref_underbrush_msg.feet.feet.at(i).velocity.z = foot_vz;

            ref_underbrush_msg.feet.feet.at(i).acceleration.x = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.y = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.z = 0;

            break;
          }
        }
      }
    }

    // DEBUG: does ikRobotState() below change a STANCE leg's joint angles,
    // even though that leg's foot position was never touched by the swing
    // loop above? ref_underbrush_msg.joints.position going into this call is
    // still exactly ref_state_msg_'s (the clean planner value) for any
    // stance leg -- the swing loop above only ever writes .feet.feet.at(i)
    // for legs with !contact. If ikRobotState() re-derives a different valid
    // IK solution for the same foot position, "before" and "after" will
    // differ for a leg that is currently in stance. Remove once the
    // ref_state_msg_ vs ref_underbrush_msg abad-target investigation is done.
    static const char* kDebugLegNames[4] = {"FL", "RL", "FR", "RR"};
    double abad_before[4], hip_before[4], knee_before[4];
    for (int i = 0; i < 4; ++i) {
      abad_before[i] = ref_underbrush_msg.joints.position.at(3 * i + 0);
      hip_before[i] = ref_underbrush_msg.joints.position.at(3 * i + 1);
      knee_before[i] = ref_underbrush_msg.joints.position.at(3 * i + 2);
    }

    quad_utils::ikRobotState(*quadKD_, ref_underbrush_msg);

    for (int i = 0; i < 4; ++i) {
      if (!ref_state_msg_.feet.feet.at(i).contact) continue;  // stance only
      double abad_after = ref_underbrush_msg.joints.position.at(3 * i + 0);
      double hip_after = ref_underbrush_msg.joints.position.at(3 * i + 1);
      double knee_after = ref_underbrush_msg.joints.position.at(3 * i + 2);
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 200,
          "[underbrush][stance-ik-drift] %s: abad before=%.4f after=%.4f "
          "delta=%.4f | hip before=%.4f after=%.4f delta=%.4f | knee "
          "before=%.4f after=%.4f delta=%.4f",
          kDebugLegNames[i], abad_before[i], abad_after,
          abad_after - abad_before[i], hip_before[i], hip_after,
          hip_after - hip_before[i], knee_before[i], knee_after,
          knee_after - knee_before[i]);
    }

    // Compute abad joint IK
    ref_abad_msg = ref_underbrush_msg;
    for (int i = 0; i < 4; i++) {
      if (!ref_state_msg_.feet.feet.at(i).contact) {
        for (size_t j = 0; j < last_local_plan_msg_->states.size() - 1; j++) {
          if (t_now < rclcpp::Time(last_local_plan_msg_->states[j].header.stamp)
                          .seconds() &&
              bool(last_local_plan_msg_->states[j].feet.feet.at(i).contact)) {
            ref_abad_msg.feet.feet.at(i).position.x =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.x;
            ref_abad_msg.feet.feet.at(i).position.y =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.y;
            ref_abad_msg.feet.feet.at(i).position.z =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.z;
            break;
          }
        }
      }
    }
    quad_utils::ikRobotState(*quadKD_, ref_abad_msg);

    // DEBUG: this abad-specific touchdown search uses the SAME plan-relative-
    // vs-absolute-timestamp bug as the main swing loop above (t_now compared
    // directly against an absolute stamp instead of stamp - t_first_state).
    // Build a second, throwaway copy using the CORRECTED comparison, run IK
    // on it too, and compare -- purely for logging, does not change what
    // actually gets commanded. If "buggy" and "fixed" diverge by something in
    // the ~0.05-0.2 rad range, that's this call site producing the abad
    // discrepancy. Remove once the abad investigation is done.
    {
      quad_msgs::msg::RobotState ref_abad_fixed_msg = ref_underbrush_msg;
      for (int i = 0; i < 4; i++) {
        if (!ref_state_msg_.feet.feet.at(i).contact) {
          for (size_t j = 0; j < last_local_plan_msg_->states.size() - 1; j++) {
            if (t_now < (rclcpp::Time(last_local_plan_msg_->states[j].header.stamp) -
                         t_first_state)
                            .seconds() &&
                bool(last_local_plan_msg_->states[j].feet.feet.at(i).contact)) {
              ref_abad_fixed_msg.feet.feet.at(i).position =
                  last_local_plan_msg_->states[j].feet.feet.at(i).position;
              break;
            }
          }
        }
      }
      quad_utils::ikRobotState(*quadKD_, ref_abad_fixed_msg);
      for (int i = 0; i < 4; ++i) {
        if (ref_state_msg_.feet.feet.at(i).contact) continue;  // swing only
        double abad_buggy = ref_abad_msg.joints.position.at(3 * i + 0);
        double abad_fixed = ref_abad_fixed_msg.joints.position.at(3 * i + 0);
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 200,
            "[underbrush][abad-stale-vs-fixed] %s: buggy=%.4f fixed=%.4f "
            "delta=%.4f",
            kDebugLegNames[i], abad_buggy, abad_fixed, abad_buggy - abad_fixed);
      }
    }

    for (int i = 0; i < num_feet_; ++i) {
      if (!ref_state_msg_.feet.feet.at(i).contact) {
        ref_underbrush_msg.joints.position.at(3 * i + 0) =
            ref_abad_msg.joints.position.at(3 * i + 0);
        ref_underbrush_msg.joints.velocity.at(3 * i + 0) = 0;
      }
    }

    for (int i = 0; i < num_feet_; ++i) {
      // Limit the joint velocities computed by inverse kinematics
      for (int j = 0; j < 3; ++j) {
        if (ref_underbrush_msg.joints.velocity.at(3 * i + j) > retract_vel_) {
          ref_underbrush_msg.joints.velocity.at(3 * i + j) = retract_vel_;
        }
        if (ref_underbrush_msg.joints.velocity.at(3 * i + j) < -retract_vel_) {
          ref_underbrush_msg.joints.velocity.at(3 * i + j) = -retract_vel_;
        }
      }

      // Push the joints out of bad configurations. Thresholds (-0.5 for hip,
      // 0.3 for knee) were written for Spirit40, whose wire frame matches its
      // URDF frame (sign=1, offset=0). Go2 wires hip with sign=-1, offset=pi/2
      // and knee with sign=1, offset=-pi, so at stand the wire-frame knee is
      // -1.5 rad -- which incorrectly trips the Spirit "knee too low" clamp
      // every tick and drives the knee against retract_vel. Do the check in
      // URDF frame instead and map the push velocity back to wire frame; this
      // is a no-op for Spirit and correct for Go2.
      {
        const double hip_sign = quadKD_->getJointSign(i, 1);
        const double hip_off  = quadKD_->getJointOriginOffset(i, 1);
        const double knee_sign = quadKD_->getJointSign(i, 2);
        const double knee_off  = quadKD_->getJointOriginOffset(i, 2);

        const double q_hip_wire  = robot_state_msg.joints.position.at(3 * i + 1);
        const double q_knee_wire = robot_state_msg.joints.position.at(3 * i + 2);
        const double q_hip_urdf  = (q_hip_wire  - hip_off)  / hip_sign;
        const double q_knee_urdf = (q_knee_wire - knee_off) / knee_sign;

        if (q_knee_urdf < 0.3) {
          const double v_push_urdf = -20 * (q_knee_urdf - 0.3);
          ref_underbrush_msg.joints.velocity.at(3 * i + 2) += v_push_urdf * knee_sign;
        }
        if (q_hip_urdf < -0.5) {
          const double v_push_urdf = -20 * (q_hip_urdf + 0.5);
          ref_underbrush_msg.joints.velocity.at(3 * i + 1) += v_push_urdf * hip_sign;
        }
      }

      // Limit the joint velocities again
      for (int j = 0; j < 3; ++j) {
        if (ref_underbrush_msg.joints.velocity.at(3 * i + j) > retract_vel_) {
          ref_underbrush_msg.joints.velocity.at(3 * i + j) = retract_vel_;
        }
        if (ref_underbrush_msg.joints.velocity.at(3 * i + j) < -retract_vel_) {
          ref_underbrush_msg.joints.velocity.at(3 * i + j) = -retract_vel_;
        }
      }
    }

    // Only apply the Underbrush swing law to legs that are actually in
    // obstructed (retract) mode. Un-obstructed swing legs keep the local
    // plan's swing trajectory, exactly as InverseDynamicsController does.
    // force_mode_ here is from the previous tick (it is updated in the
    // command loop below); the one-tick lag is well inside min_switch_.
    for (int i = 0; i < num_feet_; ++i) {
      if (!ref_state_msg_.feet.feet.at(i).contact && !force_mode_.at(i)) {
        ref_underbrush_msg.feet.feet.at(i) = ref_state_msg_.feet.feet.at(i);
        for (int j = 0; j < 3; ++j) {
          ref_underbrush_msg.joints.position.at(3 * i + j) =
              ref_state_msg_.joints.position.at(3 * i + j);
          ref_underbrush_msg.joints.velocity.at(3 * i + j) =
              ref_state_msg_.joints.velocity.at(3 * i + j);
        }
      }
    }

    ref_state_msg_ = ref_underbrush_msg;

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

    // DEBUG: (1) at the exact tick a leg transitions swing->stance, compare
    // the abad angle it's actually carrying in (robot_state_msg, real
    // hardware) against the fresh stance plan's abad target for that same
    // tick -- tests whether a swing-phase corruption shows up as a jump at
    // touchdown. (2) throttled print of the CURRENT plan's own stance abad
    // target over time, per stance leg -- tests whether the planner itself
    // has already drifted vs. this just being a tracking-only issue. Remove
    // once the abad investigation is done.
    {
      static bool was_stance[4] = {false, false, false, false};
      for (int i = 0; i < 4; ++i) {
        bool now_stance = contact_mode[i];
        if (now_stance && !was_stance[i]) {
          RCLCPP_INFO(
              node_->get_logger(),
              "[underbrush][stance-transition] %s: TOUCHDOWN incoming_actual_"
              "abad=%.4f fresh_plan_target_abad=%.4f jump=%.4f",
              kDebugLegNames[i], robot_state_msg.joints.position.at(3 * i + 0),
              ref_state_msg_.joints.position.at(3 * i + 0),
              ref_state_msg_.joints.position.at(3 * i + 0) -
                  robot_state_msg.joints.position.at(3 * i + 0));
        }
        was_stance[i] = now_stance;
        if (now_stance) {
          RCLCPP_INFO_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 200,
              "[underbrush][stance-plan-target] %s: plan_abad=%.4f "
              "actual_abad=%.4f",
              kDebugLegNames[i], ref_state_msg_.joints.position.at(3 * i + 0),
              robot_state_msg.joints.position.at(3 * i + 0));
        }
      }
    }

    // Compute joint torques
    quadKD_->computeInverseDynamics(ref_foot_acceleration, grf_array,
                                    contact_mode, tau_array);

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);

      if (contact_mode[i]) {
        // Stance phase
        for (int j = 0; j < 3; ++j) {
          int joint_idx = 3 * i + j;

          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(j)
              .pos_setpoint = ref_state_msg_.joints.position.at(joint_idx);
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(j)
              .vel_setpoint = ref_state_msg_.joints.velocity.at(joint_idx);
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(j)
              .torque_ff = tau_array(joint_idx);

          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
              stance_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
              stance_kd_.at(j);
        }
      } else {
        // Swing phase

        // Switch swing modes
        if (force_mode_.at(i) && (t_now2 - t_switch_.at(i) > min_switch_) &&
            (last_body_force_estimate_msg_->joint_torques.at(3 * i + 2) <
             tau_contact_end_) &&
            t_TD_.at(i) - t_now2 >= t_down_) {
          // leg is not obstructed anymore: stop retracting and extend
          last_mode_.at(i) = 0;
          force_mode_.at(i) = 0;
          t_switch_.at(i) = t_now2;
        } else if (!force_mode_.at(i) &&
                   (t_now2 - t_switch_.at(i) > min_switch_) &&
                   (last_body_force_estimate_msg_->joint_torques.at(3 * i + 2) >
                        tau_contact_start_ ||
                    last_body_force_estimate_msg_->joint_torques.at(3 * i + 1) >
                            tau_contact_start_ &&
                        t_now2 - t_LO_.at(i) > t_up_)) {
          // leg is now obstructed: retract it over the obstruction
          force_mode_.at(i) = 1;
          t_switch_.at(i) = t_now2;
        }

        if (force_mode_.at(i) &&
            t_TD_.at(i) - t_now2 < t_down_) {  // insufficient time left in
                                               // stance; put the foot down)
          force_mode_.at(i) = 0;
          t_switch_.at(i) = t_now2;
          last_mode_.at(i) = 1;
        }

        if (t_now2 - t_LO_.at(i) < t_up_) {
          if (last_mode_.at(i)) {
            force_mode_.at(i) = 1;  // retain previous mode
            t_switch_.at(i) = t_LO_.at(i);
          }
        }

        if (!force_mode_.at(i)) {
          // Usual swing mode
          for (int j = 0; j < 3; ++j) {
            int joint_idx = 3 * i + j;

            leg_command_array_msg.leg_commands.at(i)
                .motor_commands.at(j)
                .pos_setpoint = ref_state_msg_.joints.position.at(joint_idx);
            leg_command_array_msg.leg_commands.at(i)
                .motor_commands.at(j)
                .vel_setpoint = ref_state_msg_.joints.velocity.at(joint_idx);
            leg_command_array_msg.leg_commands.at(i)
                .motor_commands.at(j)
                .torque_ff = tau_array(joint_idx);

            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
                swing_kp_.at(j);
            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
                swing_kd_.at(j);
          }
          // DEBUG: what actually ends up commanded for abad -- closes the
          // loop from "target computed above" to "value that reaches the
          // motor". Remove once the abad investigation is done.
          RCLCPP_INFO_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 200,
              "[underbrush][abad-final-cmd] %s: pos_setpoint=%.4f "
              "vel_setpoint=%.4f kp=%.1f",
              kDebugLegNames[i],
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(0)
                  .pos_setpoint,
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(0)
                  .vel_setpoint,
              leg_command_array_msg.leg_commands.at(i).motor_commands.at(0).kp);
        } else {
          // Obstructed swing mode
          for (int j = 0; j < 3; ++j) {
            leg_command_array_msg.leg_commands.at(i)
                .motor_commands.at(j)
                .pos_setpoint = 0;
            leg_command_array_msg.leg_commands.at(i)
                .motor_commands.at(j)
                .torque_ff = 0;
            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
                0;
            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
                swing_kd_.at(j);
          }

          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(0)
              .pos_setpoint = ref_state_msg_.joints.position.at(3 * i + 0);
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(0)
              .vel_setpoint = ref_state_msg_.joints.velocity.at(3 * i + 0);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(0).kp =
              swing_kp_.at(0);

          // Retract direction: Spirit's URDF hip axis is -y so -retract_vel
          // produces the physical "lift-and-back" motion; Go2's URDF hip
          // axis is +y so the opposite sign is required for the same
          // physical direction. hip_retract_sign_ carries this per-robot.
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(1)
              .vel_setpoint = -retract_vel_ * hip_retract_sign_;

          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(2)
              .vel_setpoint = 0;
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(2).kd = 0;
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(2)
              .torque_ff = -tau_push_;
        }
      }
    }

    last_grf_array_ = grf_array;
    return true;
  }
}
