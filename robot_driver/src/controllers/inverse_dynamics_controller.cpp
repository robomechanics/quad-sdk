#include "robot_driver/controllers/inverse_dynamics_controller.hpp"

InverseDynamicsController::InverseDynamicsController(
    rclcpp::Node::SharedPtr node, const std::string& robot_ns,
    std::shared_ptr<quad_utils::QuadKD2> quadKD)
    : LegController(node, robot_ns, quadKD) {}

bool InverseDynamicsController::computeLegCommandArray(
    const quad_msgs::msg::RobotState& robot_state_msg,
    quad_msgs::msg::LegCommandArray& leg_command_array_msg,
    quad_msgs::msg::GRFArray& grf_array_msg) {
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
      // Bail without writing leg commands. Falling through into the
      // interpolation loop is unsafe: the loop's gating condition can
      // never be true when t_now is out of [front, back], so it exits
      // without populating ref_state_msg_, and downstream reads
      // (ref_state_msg_.feet.feet[i]) dereference an empty vector and
      // segfault. Throttled to 1 Hz so a stale plan doesn't flood the
      // console at the controller tick rate.
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

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
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

        if (contact_mode[i]) {
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
              stance_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
              stance_kd_.at(j);
        } else {
          leg_command_array_msg.leg_commands.at(i)
              .motor_commands.at(j)
              .torque_ff += swing_cart_fb(joint_idx);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp =
              swing_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd =
              swing_kd_.at(j);
        }
      }
    }

    last_grf_array_ = grf_array;
    return true;
  }
}
