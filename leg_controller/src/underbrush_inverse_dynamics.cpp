#include "leg_controller/underbrush_inverse_dynamics.h"

UnderbrushInverseDynamicsController::UnderbrushInverseDynamicsController() {
  quadKD_ = std::make_shared<quad_utils::QuadKD>();

  force_mode_ = {0, 0, 0, 0};

  double t_now = ros::Time::now().toSec();
  t_switch_ = {t_now, t_now, t_now, t_now};
  t_LO_ = {t_now, t_now, t_now, t_now};
  t_TD_ = {t_now, t_now, t_now, t_now};
}

void UnderbrushInverseDynamicsController::setGains(std::vector<double> stance_kp, std::vector<double> stance_kd,
  std::vector<double> swing_kp, std::vector<double> swing_kd) {

  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
}

void UnderbrushInverseDynamicsController::computeLegCommandArrayFromPlan(
  const quad_msgs::RobotState::ConstPtr &robot_state_msg,
  const quad_msgs::RobotPlan::ConstPtr &local_plan_msg,
  quad_msgs::LegCommandArray &leg_command_array_msg,
  quad_msgs::GRFArray &grf_array_msg,
  const quad_msgs::BodyForceEstimate::ConstPtr& body_force_estimate_msg)
{
  leg_command_array_msg.leg_commands.resize(num_feet_);

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
  quad_utils::vectorToEigen(robot_state_msg->joints.position, joint_positions);
  quad_utils::vectorToEigen(robot_state_msg->joints.velocity, joint_velocities);
  body_state = quad_utils::bodyStateMsgToEigen(robot_state_msg->body);

  // Define vectors for state positions and velocities 
  Eigen::VectorXd state_positions(3*num_feet_+6), state_velocities(3*num_feet_+6);
  state_positions << joint_positions, body_state.head(6);
  state_velocities << joint_velocities, body_state.tail(6);

  // Initialize variables for ff and fb
  quad_msgs::RobotState ref_state_msg, ref_underbrush_msg;
  Eigen::VectorXd tau_array(3*num_feet_), tau_swing_leg_array(3*num_feet_);

  // Get reference state and grf from local plan or traj + grf messages
  double t_now = ros::Time::now().toSec();
  
  if ( (t_now < local_plan_msg->states.front().header.stamp.toSec()) || 
        (t_now > local_plan_msg->states.back().header.stamp.toSec()) ) {
    ROS_ERROR("ID node couldn't find the correct ref state!");
  }

  // Interpolate the local plan to get the reference state and ff GRF
  for (int i = 0; i < local_plan_msg->states.size()-1; i++) {
    
    if ( (t_now >= local_plan_msg->states[i].header.stamp.toSec()) && 
        ( t_now <  local_plan_msg->states[i+1].header.stamp.toSec() )) {

      double t_interp = (t_now - local_plan_msg->states[i].header.stamp.toSec())/
        (local_plan_msg->states[i+1].header.stamp.toSec() - 
          local_plan_msg->states[i].header.stamp.toSec());
      
      quad_utils::interpRobotState(local_plan_msg->states[i],
        local_plan_msg->states[i+1], t_interp, ref_state_msg);

      // quad_utils::interpGRFArray(local_plan_msg->grfs[i],
      //   local_plan_msg->grfs[i+1], t_interp, grf_array_msg);

      // ref_state_msg = local_plan_msg->states[i];
      grf_array_msg = local_plan_msg->grfs[i];

      // Don't switch immediately after beginning swing
      for (int j = 0; j < num_feet_; j++) {
        if (local_plan_msg->states[i].feet.feet.at(j).contact && 
            !local_plan_msg->states[i+1].feet.feet.at(j).contact) {
          //t_switch_.at(j) = local_plan_msg->states[i+1].header.stamp.toSec();
          t_LO_.at(j) = local_plan_msg->states[i+1].header.stamp.toSec();
        }
      }

      break;
    }
  }

  // Underbrush swing leg motion
  ref_underbrush_msg = ref_state_msg;
  for (int i = 0; i < 4; i++) {
    if (!ref_state_msg.feet.feet.at(i).contact){
      for (int j = 0; j < local_plan_msg->states.size()-1; j++) {
        if (t_now < local_plan_msg->states[j].header.stamp.toSec() &&
            bool(local_plan_msg->states[j].feet.feet.at(i).contact)) {
          ref_underbrush_msg.feet.feet.at(i).position.x = local_plan_msg->states[j].feet.feet.at(i).position.x;
          ref_underbrush_msg.feet.feet.at(i).position.y = local_plan_msg->states[j].feet.feet.at(i).position.y;
          ref_underbrush_msg.feet.feet.at(i).position.z = local_plan_msg->states[j].feet.feet.at(i).position.z;
          ref_underbrush_msg.feet.feet.at(i).velocity.x = 0;
          ref_underbrush_msg.feet.feet.at(i).velocity.y = 0;
          ref_underbrush_msg.feet.feet.at(i).velocity.z = 0;
          ref_underbrush_msg.feet.feet.at(i).acceleration.x = 0;
          ref_underbrush_msg.feet.feet.at(i).acceleration.y = 0;
          ref_underbrush_msg.feet.feet.at(i).acceleration.z = 0;

          break;
        }
      }
    }
  }
  quad_utils::ikRobotState(*quadKD_, ref_underbrush_msg);
  for (int i = 0; i < 4; i++) {
    int abad_idx = 3*i+0;
    int hip_idx = 3*i+1;
    int knee_idx = 3*i+2;
    ref_underbrush_msg.joints.position.at(knee_idx) +=
      -0.7*std::abs(state_positions[hip_idx] - ref_underbrush_msg.joints.position.at(hip_idx))
      -0.6*std::abs(state_positions[abad_idx] - ref_underbrush_msg.joints.position.at(abad_idx));
    if (ref_underbrush_msg.joints.position.at(knee_idx) < 0.3) {
      ref_underbrush_msg.joints.position.at(knee_idx) = 0.3;
    }
  }
  ref_state_msg = ref_underbrush_msg;

  // Declare plan and state data as Eigen vectors
  Eigen::VectorXd ref_body_state(12), grf_array(3 * num_feet_),
      ref_foot_positions(3 * num_feet_), ref_foot_velocities(3 * num_feet_), ref_foot_acceleration(3 * num_feet_);

  // Load plan and state data from messages
  ref_body_state = quad_utils::bodyStateMsgToEigen(ref_state_msg.body);
  quad_utils::multiFootStateMsgToEigen(
      ref_state_msg.feet, ref_foot_positions, ref_foot_velocities, ref_foot_acceleration);
  grf_array = quad_utils::grfArrayMsgToEigen(grf_array_msg);

  // Load contact mode
  std::vector<int> contact_mode(num_feet_);
  for (int i = 0; i < num_feet_; i++) {
    contact_mode[i] = ref_state_msg.feet.feet[i].contact;
  }
  
  // Compute joint torques
  quadKD_->computeInverseDynamics(state_positions, state_velocities, ref_foot_acceleration,grf_array,
    contact_mode, tau_array);

  //ROS_INFO_THROTTLE(0.1, "tau_k 0: %f", body_force_estimate_msg->body_wrenches.at(0).torque.z);

  for (int i = 0; i < num_feet_; ++i) {
    leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);

    // Time since last switch

    // Switch swing modes
    if (force_mode_.at(i) && (t_now - t_switch_.at(i) > 0.05) && //JYTODO: make a parameter
        (body_force_estimate_msg->body_wrenches.at(i).torque.z < 0.25)) { //JYTODO: make a parameter
      force_mode_.at(i) = 0;
      t_switch_.at(i) = t_now;
    } else if (!force_mode_.at(i) && (t_now - t_switch_.at(i) > 0.05) && 
          body_force_estimate_msg->body_wrenches.at(i).torque.z > 1.0) {
      force_mode_.at(i) = 1;
      t_switch_.at(i) = t_now;
      ROS_INFO_STREAM("OBSTRUCTION DETECTED");
    }
    //force_mode_.at(i) = 0;

    if (contact_mode[i]) {
      // Stance phase
      for (int j = 0; j < 3; ++j) {

        int joint_idx = 3*i+j;

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          ref_state_msg.joints.position.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 
          ref_state_msg.joints.velocity.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff =
            tau_array(joint_idx);

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = stance_kp_.at(j);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = stance_kd_.at(j);
      }
    } else {
      // Swing phase
      if (!force_mode_.at(i)) {
        // Usual swing mode
        for (int j = 0; j < 3; ++j) {

          int joint_idx = 3*i+j;

          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
            ref_state_msg.joints.position.at(joint_idx);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 
            ref_state_msg.joints.velocity.at(joint_idx);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;

          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = swing_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = swing_kd_.at(j);
        }
      } else{
        // Obstructed swing mode
        ROS_INFO_THROTTLE(0.01, "Attempting circumvention");
        for (int j = 0; j < 3; ++j) {
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 0;
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff = 0;
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = 0;
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = swing_kd_.at(j);
        }

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(0).pos_setpoint = 
          ref_state_msg.joints.position.at(3*i+0);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(0).vel_setpoint = 
          ref_state_msg.joints.velocity.at(3*i+0);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(0).kp = swing_kp_.at(0);

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(1).vel_setpoint = -20; // JYTODO: make parameter

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(2).vel_setpoint = 0;
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(2).kd = 0;
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(2).torque_ff = -1.0;//-0.5; // JYTODO: make parameter
      }
    }
  }
  
}