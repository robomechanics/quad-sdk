#include "leg_controller/inverse_dynamics.h"

InverseDynamicsController::InverseDynamicsController() {}

bool InverseDynamicsController::computeLegCommandArray(
  const quad_msgs::RobotState::ConstPtr &robot_state_msg,
  quad_msgs::LegCommandArray &leg_command_array_msg,
  quad_msgs::GRFArray &grf_array_msg)
{
  
  if ((last_local_plan_msg_ == NULL) || 
    ((ros::Time::now() - last_local_plan_msg_->header.stamp).toSec() >= 0.1)) {
    
    return false;
  } else {

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
    quad_msgs::RobotState ref_state_msg;
    Eigen::VectorXd tau_array(3*num_feet_), tau_swing_leg_array(3*num_feet_);

    // Get reference state and grf from local plan or traj + grf messages
    ros::Time t_first_state = last_local_plan_msg_->states.front().header.stamp;
    double t_now = (ros::Time::now() - last_local_plan_msg_->state_timestamp).toSec(); // Use time of state - RECOMMENDED
    // double t_now = (ros::Time::now() - last_local_plan_time_).toSec(); // Use time of plan receipt
    // double t_now = (ros::Time::now() - t_first_state).toSec(); // Use time of first state in plan

    if ( (t_now < (last_local_plan_msg_->states.front().header.stamp - t_first_state).toSec()) || 
          (t_now > (last_local_plan_msg_->states.back().header.stamp - t_first_state).toSec()) ) {
      ROS_ERROR("ID node couldn't find the correct ref state!");
    }    

    // Interpolate the local plan to get the reference state and ff GRF
    for (int i = 0; i < last_local_plan_msg_->states.size()-1; i++) {
      
      if ( (t_now >= (last_local_plan_msg_->states[i].header.stamp - t_first_state).toSec()) && 
          ( t_now <  (last_local_plan_msg_->states[i+1].header.stamp - t_first_state).toSec() )) {

        double t_interp = (t_now - (last_local_plan_msg_->states[i].header.stamp-t_first_state).toSec())/
          (last_local_plan_msg_->states[i+1].header.stamp.toSec() - 
            last_local_plan_msg_->states[i].header.stamp.toSec());
        
        quad_utils::interpRobotState(last_local_plan_msg_->states[i],
          last_local_plan_msg_->states[i+1], t_interp, ref_state_msg);

        // quad_utils::interpGRFArray(last_local_plan_msg_->grfs[i],
        //   last_local_plan_msg_->grfs[i+1], t_interp, grf_array_msg);

        // ref_state_msg = last_local_plan_msg_->states[i];
        grf_array_msg = last_local_plan_msg_->grfs[i];

        break;
      }
    }

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

    for (int i = 0; i < num_feet_; ++i) {
      leg_command_array_msg.leg_commands.at(i).motor_commands.resize(3);
      for (int j = 0; j < 3; ++j) {

        int joint_idx = 3*i+j;

        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).pos_setpoint = 
          ref_state_msg.joints.position.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).vel_setpoint = 
          ref_state_msg.joints.velocity.at(joint_idx);
        leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).torque_ff =
            tau_array(joint_idx);

        if (contact_mode[i]) {
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = stance_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = stance_kd_.at(j);
        } else {
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kp = swing_kp_.at(j);
          leg_command_array_msg.leg_commands.at(i).motor_commands.at(j).kd = swing_kd_.at(j);
        }
      }
    }

    return true;
  }
}