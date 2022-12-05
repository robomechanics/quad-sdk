#include "robot_driver/controllers/underbrush_inverse_dynamics.h"

UnderbrushInverseDynamicsController::UnderbrushInverseDynamicsController() {
  force_mode_ = {0, 0, 0, 0};
  last_mode_ = {0, 0, 0, 0};

  double t_now = ros::Time::now().toSec();
  t_switch_ = {t_now, t_now, t_now, t_now};
  t_LO_ = {t_now, t_now, t_now, t_now};
  t_TD_ = {t_now, t_now, t_now, t_now};
}

void UnderbrushInverseDynamicsController::updateBodyForceEstimate(
    const quad_msgs::BodyForceEstimate::ConstPtr &msg) {
  last_body_force_estimate_msg_ = msg;
}

void UnderbrushInverseDynamicsController::setUnderbrushParams(
    double retract_vel, double tau_push, double tau_contact_start,
    double tau_contact_end, double min_switch, double t_down, double t_up) {
  retract_vel_ = retract_vel;
  tau_push_ = tau_push;
  tau_contact_start_ = tau_contact_start;
  tau_contact_end_ = tau_contact_end;
  min_switch_ = min_switch;
  t_down_ = t_down;
  t_up_ = t_up;
}

bool UnderbrushInverseDynamicsController::computeLegCommandArray(
    const quad_msgs::RobotState &robot_state_msg,
    quad_msgs::LegCommandArray &leg_command_array_msg,
    quad_msgs::GRFArray &grf_array_msg) {
  if ((last_local_plan_msg_ == NULL || last_body_force_estimate_msg_ == NULL) ||
      ((ros::Time::now() - last_local_plan_msg_->header.stamp).toSec() >=
       0.1)) {
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
    quad_msgs::RobotState ref_underbrush_msg;
    Eigen::VectorXd tau_array(3 * num_feet_),
        tau_swing_leg_array(3 * num_feet_);

    // Get reference state and grf from local plan or traj + grf messages
    ros::Time t_first_state = last_local_plan_msg_->states.front().header.stamp;
    double t_now = (ros::Time::now() - last_local_plan_msg_->state_timestamp)
                       .toSec();  // Use time of state - RECOMMENDED
    double t_now2 = ros::Time::now().toSec();
    // double t_now = (ros::Time::now() - last_local_plan_time_).toSec(); // Use
    // time of plan receipt double t_now = (ros::Time::now() -
    // t_first_state).toSec(); // Use time of first state in plan

    if ((t_now <
         (last_local_plan_msg_->states.front().header.stamp - t_first_state)
             .toSec()) ||
        (t_now >
         (last_local_plan_msg_->states.back().header.stamp - t_first_state)
             .toSec())) {
      ROS_ERROR("ID node couldn't find the correct ref state!");
    }

    int all_TD = 0;  // end looping when all next touchdowns have been found

    // Interpolate the local plan to get the reference state and ff GRF
    for (int i = 0; i < last_local_plan_msg_->states.size() - 1; i++) {
      if ((t_now >=
           (last_local_plan_msg_->states[i].header.stamp - t_first_state)
               .toSec()) &&
          (t_now <
           (last_local_plan_msg_->states[i + 1].header.stamp - t_first_state)
               .toSec())) {
        double t_interp =
            (t_now -
             (last_local_plan_msg_->states[i].header.stamp - t_first_state)
                 .toSec()) /
            (last_local_plan_msg_->states[i + 1].header.stamp.toSec() -
             last_local_plan_msg_->states[i].header.stamp.toSec());

        // Linearly interpolate between states
        quad_utils::interpRobotState(last_local_plan_msg_->states[i],
                                     last_local_plan_msg_->states[i + 1],
                                     t_interp, ref_state_msg_);

        // quad_utils::interpGRFArray(last_local_plan_msg_->grfs[i],
        //   last_local_plan_msg_->grfs[i+1], t_interp, grf_array_msg);

        // ref_state_msg = last_local_plan_msg_->states[i];
        grf_array_msg = last_local_plan_msg_->grfs[i];

        // Don't switch immediately after beginning swing
        for (int j = 0; j < num_feet_; j++) {
          if (last_local_plan_msg_->states[i].feet.feet.at(j).contact &&
              !last_local_plan_msg_->states[i + 1].feet.feet.at(j).contact) {
            // t_switch_.at(j) =
            // last_local_plan_msg_->states[i+1].header.stamp.toSec();
            t_LO_.at(j) =
                last_local_plan_msg_->states[i + 1].header.stamp.toSec();
          }
        }

        // break;

      } else if (t_now < last_local_plan_msg_->states[i + 1]
                             .header.stamp.toSec()) {  // find next touchdowns
        if (all_TD == pow(2.0, num_feet_) -
                          1) {  // terminate when all next touchdowns are found
          break;
        }
        for (int j = 0; j < num_feet_; j++) {
          if (!(all_TD & (1 << j)) &&
              i > 0) {  // touchdown not yet found for this foot
            // ROS_INFO("%u, %u",i, j);
            if (ref_state_msg_.feet.feet.at(j).contact) {
              all_TD =
                  all_TD |
                  (1 << j);  // foot is in stance, don't need next touchdown
            } else {
              if (last_local_plan_msg_->states[i].feet.feet.at(j).contact) {
                t_TD_.at(j) =
                    last_local_plan_msg_->states[i].header.stamp.toSec();
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
          if (t_now < last_local_plan_msg_->states[j].header.stamp.toSec() &&
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

            // cartesian foot velocity commands
            foot_vx = 10.0 * foot_x_err;
            foot_vx =
                abs(foot_vx) > 3.0 ? (foot_vx > 0 ? 1 : -1) * 3.0 : foot_vx;
            foot_vy = 10.0 * foot_y_err;
            foot_vy =
                abs(foot_vy) > 3.0 ? (foot_vy > 0 ? 1 : -1) * 3.0 : foot_vy;
            foot_vz = 10.0 * foot_z_err;

            foot_horz_err = sqrt(foot_x_err * foot_x_err);
            if (foot_horz_err > 0.02 && t_TD_.at(i) - t_now2 > t_down_) {
              // don't put the foot down unless it's close to the right x, y
              // position or there's no time left
              foot_vz = 1.0 / (100 * (foot_horz_err - 0.02) + 1) * foot_vz;
            }

            foot_z_hip = robot_state_msg.feet.feet.at(i).position.z -
                         robot_state_msg.body.pose.position.z;
            if (foot_z_hip > -0.05) {
              // foot is too high above hip; singularity problems
              foot_vx = 1 / (50 * (foot_z_hip + 0.05) + 1) * foot_vx;
              foot_vy = 0;
              foot_vz += -10.0 * (foot_z_hip - 0.05);
            }
            foot_vz =
                abs(foot_vz) > 3.0 ? (foot_vz > 0 ? 1 : -1) * 3.0 : foot_vz;

            ref_underbrush_msg.feet.feet.at(i).velocity.x = foot_vx;
            ref_underbrush_msg.feet.feet.at(i).velocity.y = foot_vy;
            ref_underbrush_msg.feet.feet.at(i).velocity.z = foot_vz;

            ref_underbrush_msg.feet.feet.at(i).acceleration.x = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.y = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.z = 0;
            /*
            ref_underbrush_msg.feet.feet.at(i).position.x =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.x;
            ref_underbrush_msg.feet.feet.at(i).position.y =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.y;
            ref_underbrush_msg.feet.feet.at(i).position.z =
                last_local_plan_msg_->states[j].feet.feet.at(i).position.z;
            ref_underbrush_msg.feet.feet.at(i).velocity.x = 0;
            ref_underbrush_msg.feet.feet.at(i).velocity.y = 0;
            ref_underbrush_msg.feet.feet.at(i).velocity.z = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.x = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.y = 0;
            ref_underbrush_msg.feet.feet.at(i).acceleration.z = 0;
            */
            break;
          }
        }
      }
    }
    quad_utils::ikRobotState(*quadKD_, ref_underbrush_msg);
    for (int i = 0; i < num_feet_; ++i) {
      // Push the joints out of bad configurations
      if (robot_state_msg.joints.position.at(3 * i + 2) < 0.3) {
        ref_underbrush_msg.joints.velocity.at(3 * i + 2) +=
            -20 * (robot_state_msg.joints.position.at(3 * i + 2) - 0.3);
      }
      if (robot_state_msg.joints.position.at(3 * i + 1) < -0.5) {
        ref_underbrush_msg.joints.velocity.at(3 * i + 1) +=
            -20 * (robot_state_msg.joints.position.at(3 * i + 1) + 0.5);
      }
      for (int j = 0; j < 3; ++j) {
        if (ref_underbrush_msg.joints.velocity.at(3 * i + j) > retract_vel_) {
          ref_underbrush_msg.joints.velocity.at(3 * i + j) = retract_vel_;
        }
        if (ref_underbrush_msg.joints.velocity.at(3 * i + j) < -retract_vel_) {
          ref_underbrush_msg.joints.velocity.at(3 * i + j) = -retract_vel_;
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

    // Compute joint torques
    quadKD_->computeInverseDynamics(state_positions, state_velocities,
                                    ref_foot_acceleration, grf_array,
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

        /*
        ROS_INFO("C %u, t %f, hip %2.3f, knee %2.3f", force_mode_.at(i),
          t_now2 - t_switch_.at(i),
          last_body_force_estimate_msg_->body_wrenches.at(i).torque.y,
          last_body_force_estimate_msg_->body_wrenches.at(i).torque.z);
        */

        // ROS_INFO("%1.3f, %1.3f, %1.3f, %1.3f, %1.3f, %1.3f", retract_vel_,
        // tau_push_, tau_contact_start_, tau_contact_end_, min_switch_,
        // t_down_);

        // Switch swing modes
        if (force_mode_.at(i) && (t_now2 - t_switch_.at(i) > min_switch_) &&
            (last_body_force_estimate_msg_->body_wrenches.at(i).torque.z <
             tau_contact_end_) &&
            t_TD_.at(i) - t_now2 >= t_down_) {
          last_mode_.at(i) = 0;
          force_mode_.at(i) = 0;
          t_switch_.at(i) = t_now2;
        } else if (!force_mode_.at(i) &&
                   (t_now2 - t_switch_.at(i) > min_switch_) &&
                   (last_body_force_estimate_msg_->body_wrenches.at(i)
                            .torque.z > tau_contact_start_ ||
                    last_body_force_estimate_msg_->body_wrenches.at(i)
                                .torque.y > tau_contact_start_ &&
                        t_now2 - t_LO_.at(i) > t_up_)) {
          force_mode_.at(i) = 1;
          t_switch_.at(i) = t_now2;
          // ROS_INFO_STREAM("OBSTRUCTION DETECTED");
        }
        // force_mode_.at(i) = 0;

        if (force_mode_.at(i) &&
            t_TD_.at(i) - t_now2 < t_down_) {  // insufficient time left in
                                               // stance; put the foot down)
          force_mode_.at(i) = 0;
          t_switch_.at(i) = t_now2;
          last_mode_.at(i) = 1;
          ROS_INFO("Leg %u stuck at end", i);
        }

        if (t_now2 - t_LO_.at(i) < t_up_) {
          if (last_mode_.at(i)) {
            force_mode_.at(i) = 1;  // retain previous mode
            t_switch_.at(i) = t_LO_.at(i);
            ROS_INFO("Leg %u was stuck at end", i);
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
                .torque_ff = 0;

            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
                swing_kp_.at(j);
            leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
                swing_kd_.at(j);
          }
        } else {
          // Obstructed swing mode
          // ROS_INFO_THROTTLE(0.01, "Attempting circumvention");
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

          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(1)
              .vel_setpoint = -retract_vel_;
          /*
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(1)
              .torque_ff =
              robot_state_msg.joints.position.at(3 * i + 1) < -0.8
                  ? -10 * retract_vel_ *
                            (robot_state_msg.joints.position.at(3 * i + 1) -
                             -0.8) -
                        0 * robot_state_msg.joints.velocity.at(3 * i + 1)
                  : 0;
          */

          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(2)
              .vel_setpoint = 0;
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(2).kd = 0;
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(2)
              .torque_ff = -tau_push_;
          /*
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(2)
              .torque_ff =
              -tau_push_ +
              (robot_state_msg.joints.position.at(3 * i + 2) < 0.1
                   ? -2 * tau_push_ *
                             (robot_state_msg.joints.position.at(3 * i + 2) -
                              0.1) -
                         0 * robot_state_msg.joints.velocity.at(3 * i + 2)
                   : 0);
          */
        }

        // Soft joint limits
        if (robot_state_msg.joints.position.at(3 * i + 2) < 0.2) {
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(2)
              .torque_ff +=
              -5 * (robot_state_msg.joints.position.at(3 * i + 2) - 0.2);
        }
        if (robot_state_msg.joints.position.at(3 * i + 1) < -0.7) {
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(1)
              .torque_ff +=
              -5 * (robot_state_msg.joints.position.at(3 * i + 1) + 0.7);
        }
      }
    }

    last_grf_array_ = grf_array;
    return true;
  }
}
