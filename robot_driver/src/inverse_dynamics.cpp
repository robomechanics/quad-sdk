#include "robot_driver/inverse_dynamics.h"

InverseDynamicsController::InverseDynamicsController() {}

bool InverseDynamicsController::computeLegCommandArray(
    const quad_msgs::RobotState &robot_state_msg,
    quad_msgs::LegCommandArray &leg_command_array_msg,
    quad_msgs::GRFArray &grf_array_msg) {
  if ((last_local_plan_msg_ == NULL) || (last_grf_sensor_msg_ == NULL)) {
    // I delete the timeout condition since when the warmstart is broken it
    // might timeout a lot
    return false;
  } else {
    leg_command_array_msg.leg_commands.resize(num_feet_);

    // Define vectors for joint positions and velocities
    Eigen::VectorXd joint_positions(3 * num_feet_),
        joint_velocities(3 * num_feet_), body_state(12);
    quad_utils::vectorToEigen(robot_state_msg.joints.position, joint_positions);
    quad_utils::vectorToEigen(robot_state_msg.joints.velocity,
                              joint_velocities);
    body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg.body);

    // Define vectors for state positions and velocities
    Eigen::VectorXd state_positions(3 * num_feet_ + 6),
        state_velocities(3 * num_feet_ + 6);
    state_positions << joint_positions, body_state.head(6);
    state_velocities << joint_velocities, body_state.tail(6);

    // Initialize variables for ff and fb
    quad_msgs::RobotState ref_state_msg;
    Eigen::VectorXd tau_array(3 * num_feet_),
        tau_swing_leg_array(3 * num_feet_);

    // Get reference state and grf from local plan or traj + grf messages
    ros::Time t_first_state = last_local_plan_msg_->states.front().header.stamp;
    double t_now = (ros::Time::now() - last_local_plan_msg_->state_timestamp)
                       .toSec();  // Use time of state - RECOMMENDED
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

        quad_utils::interpRobotState(last_local_plan_msg_->states[i],
                                     last_local_plan_msg_->states[i + 1],
                                     t_interp, ref_state_msg);

        // quad_utils::interpGRFArray(last_local_plan_msg_->grfs[i],
        //   last_local_plan_msg_->grfs[i+1], t_interp, grf_array_msg);

        // ref_state_msg = last_local_plan_msg_->states[i];
        grf_array_msg = last_local_plan_msg_->grfs[i];

        break;
      }
    }

    // Declare plan and state data as Eigen vectors
    Eigen::VectorXd ref_body_state(12), grf_array(3 * num_feet_),
        ref_foot_positions(3 * num_feet_), ref_foot_velocities(3 * num_feet_),
        ref_foot_acceleration(3 * num_feet_),
        current_foot_positions(3 * num_feet_),
        current_foot_velocities(3 * num_feet_),
        current_foot_acceleration(3 * num_feet_);

    // Load current foot data
    quad_utils::multiFootStateMsgToEigen(
        robot_state_msg.feet, current_foot_positions, current_foot_velocities,
        current_foot_acceleration);

    // Load plan and state data from messages
    ref_body_state = quad_utils::bodyStateMsgToEigen(ref_state_msg.body);
    quad_utils::multiFootStateMsgToEigen(ref_state_msg.feet, ref_foot_positions,
                                         ref_foot_velocities,
                                         ref_foot_acceleration);
    grf_array = quad_utils::grfArrayMsgToEigen(grf_array_msg);
    if (last_grf_array_.norm() >= 1e-3) {
      grf_array = grf_exp_filter_const_ * grf_array.array() +
                  (1 - grf_exp_filter_const_) * last_grf_array_.array();
      quad_utils::eigenToGRFArrayMsg(grf_array, ref_state_msg.feet,
                                     grf_array_msg);
    }

    // Load contact mode
    std::vector<int> contact_mode(num_feet_);
    for (int i = 0; i < num_feet_; i++) {
      contact_mode[i] = ref_state_msg.feet.feet[i].contact;
    }

    // Update foot IK
    ref_state_msg.body = robot_state_msg.body;
    quad_utils::ikRobotState(*quadKD_, ref_state_msg);

    // Define pd gain list
    std::vector<std::vector<double>> kp_list, kd_list;
    kp_list.resize(4);
    kd_list.resize(4);

    for (size_t i = 0; i < 4; i++) {
      // Define state machine running state
      bool contact_state_machine_done = false;

      while (!contact_state_machine_done) {
        switch (contact_state_machine_.data.at(i)) {
          case STANCE:
            if (!contact_mode[i]) {
              // Switch to swing
              contact_state_machine_.data.at(i) = SWING;
              break;
            }

            if (!get_new_plan_after_recovering_.at(i)) {
              // Still do retraction if we've not yet receive the latest plan
              // Assign retraction position
              ref_state_msg.joints.position.at(3 * i + 0) = 0.707;
              ref_state_msg.joints.position.at(3 * i + 1) = 1.4137;
              ref_state_msg.joints.position.at(3 * i + 2) = 2 * 1.4137;

              // Assign retraction velocity
              for (size_t j = 0; j < 3; j++) {
                ref_state_msg.joints.velocity.at(3 * i + j) = 0;
              }

              // Zero feedforward grf
              grf_array.segment(3 * i, 3) << 0, 0, 0;

              // Apply gains
              kp_list.at(i) = retraction_kp_;
              kd_list.at(i) = retraction_kd_;
            } else {
              // Apply gains
              kp_list.at(i) = stance_kp_;
              kd_list.at(i) = stance_kd_;
            }

            contact_state_machine_done = true;
            break;

          case SWING:
            if (contact_mode[i]) {
              // Clock switch to stance, extend first
              contact_state_machine_.data.at(i) = EXTEND;
              break;
            }

            // Apply gains
            kp_list.at(i) = swing_kp_;
            kd_list.at(i) = swing_kd_;

            contact_state_machine_done = true;
            break;

          case EXTEND:
            if (last_grf_sensor_msg_->contact_states.at(i) &&
                last_grf_sensor_msg_->vectors.at(i).z >= 5) {
              // Switch to stance if contact sensor reports it
              contact_state_machine_.data.at(i) = STANCE;
              break;
            }

            // Compute foot height relative to body
            if (current_foot_positions(3 * i + 2) - body_state(2) < -0.295) {
              // Switch to retraction if foot getting too far
              contact_state_machine_.data.at(i) = RETRACTION;
              break;
            }

            // Apply gains
            kp_list.at(i) = extend_kp_;
            kd_list.at(i) = extend_kd_;

            contact_state_machine_done = true;
            break;

          case RETRACTION:
            if (last_grf_sensor_msg_->contact_states.at(i) &&
                last_grf_sensor_msg_->vectors.at(i).z >= 5) {
              // Switch to stance if contact sensor reports it
              contact_state_machine_.data.at(i) = STANCE;
              break;
            }

            // Assign retraction position
            ref_state_msg.joints.position.at(3 * i + 0) = 0.707;
            ref_state_msg.joints.position.at(3 * i + 1) = 1.4137;
            ref_state_msg.joints.position.at(3 * i + 2) = 2 * 1.4137;

            // Assign retraction velocity
            for (size_t j = 0; j < 3; j++) {
              ref_state_msg.joints.velocity.at(3 * i + j) = 0;
            }

            // Clear new plan flag
            get_new_plan_after_recovering_.at(i) = false;

            // Zero feedforward grf
            grf_array.segment(3 * i, 3) << 0, 0, 0;

            // Apply gains
            kp_list.at(i) = retraction_kp_;
            kd_list.at(i) = retraction_kd_;

            contact_state_machine_done = true;
            break;

          default:
            break;
        }
      }
    }

    // Compute joint torques
    quadKD_->computeInverseDynamics(state_positions, state_velocities,
                                    ref_foot_acceleration, grf_array,
                                    contact_mode, tau_array);

    for (size_t i = 0; i < 4; i++) {
      if (contact_state_machine_.data.at(i) == RETRACTION) {
        tau_array.segment(3 * i, 3) << 0, 0, 0;
      }

      leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {
        int joint_idx = 3 * i + j;

        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(j)
            .pos_setpoint = ref_state_msg.joints.position.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(j)
            .vel_setpoint = ref_state_msg.joints.velocity.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i)
            .motor_commands.at(j)
            .torque_ff = tau_array(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
            kp_list.at(i).at(j);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
            kd_list.at(i).at(j);
      }
    }

    last_grf_array_ = grf_array;
    return true;
  }
}
