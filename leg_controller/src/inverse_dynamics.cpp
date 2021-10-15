#include "leg_controller/inverse_dynamics.h"

InverseDynamicsController::InverseDynamicsController() {
  quadKD_ = std::make_shared<spirit_utils::QuadKD>();
}

void InverseDynamicsController::setGains(std::vector<double> stance_kp, std::vector<double> stance_kd,
  std::vector<double> swing_kp, std::vector<double> swing_kd) {

  stance_kp_ = stance_kp;
  stance_kd_ = stance_kd;
  swing_kp_ = swing_kp;
  swing_kd_ = swing_kd;
}

void InverseDynamicsController::computeJointTorques(const Eigen::VectorXd &state_positions,
  const Eigen::VectorXd &state_velocities, const Eigen::VectorXd &grf_array,
  const Eigen::VectorXd &ref_foot_acceleration, const std::vector<int> &contact_mode,
  Eigen::VectorXd &tau_array) {

  // // Compute Jacobians
  // Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3*num_feet_, state_velocities.size());
  // quadKD_->getJacobianBodyAngVel(state_positions,jacobian);

  // // Use Jacobian and RBDL to map GRFs to joint torques
  // tau_array = -jacobian.transpose().block<12,12>(0,0)*grf_array;

  // // Use RBDL to map foot accelerations to joint torques
  // Eigen::VectorXd tau_swing_leg_array(12);
  // quadKD_->compInvDyn(state_positions, state_velocities, ref_foot_acceleration,
  //   grf_array, tau_swing_leg_array);

  // // Apply swing leg feedforward term for legs not in contact
  // for (int i = 0; i < num_feet_; i++) {
  //   if (!contact_mode[i]) {
  //     for (int j = 0; j < 3; j++) {
  //       tau_array[3*i+j] = tau_swing_leg_array[3*i+j];
  //     }
  //   }
  // }
}

void InverseDynamicsController::computeJointTorques(const Eigen::VectorXd &state_positions,
  const Eigen::VectorXd &state_velocities, const Eigen::VectorXd &grf_array,
  Eigen::VectorXd &tau_array) {

  // // Set all feet to be in contact (ignore swing phase ID)
  // Eigen::VectorXd ref_foot_acceleration(12);
  // ref_foot_acceleration.setZero();
  // std::vector<int> contact_mode = {1,1,1,1};

  // // Compute joint torques
  // computeJointTorques(state_positions, state_velocities, grf_array, ref_foot_acceleration,
  //   contact_mode, tau_array);

}

void InverseDynamicsController::computeLegCommandArrayFromPlan(
  const spirit_msgs::RobotState::ConstPtr &robot_state_msg,
  const spirit_msgs::RobotPlan::ConstPtr &local_plan_msg,
  spirit_msgs::LegCommandArray &leg_command_array_msg,
  spirit_msgs::GRFArray &grf_array_msg)
{
  leg_command_array_msg.leg_commands.resize(num_feet_);

  // Define vectors for joint positions and velocities
  Eigen::VectorXd joint_positions(3*num_feet_), joint_velocities(3*num_feet_), body_state(12);
  spirit_utils::vectorToEigen(robot_state_msg->joints.position, joint_positions);
  spirit_utils::vectorToEigen(robot_state_msg->joints.velocity, joint_velocities);
  body_state = spirit_utils::bodyStateMsgToEigen(robot_state_msg->body);

  // Define vectors for state positions and velocities 
  Eigen::VectorXd state_positions(3*num_feet_+6), state_velocities(3*num_feet_+6);
  state_positions << joint_positions, body_state.head(6);
  state_velocities << joint_velocities, body_state.tail(6);

  // Initialize variables for ff and fb
  spirit_msgs::RobotState ref_state_msg;
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
      
      spirit_utils::interpRobotState(local_plan_msg->states[i],
        local_plan_msg->states[i+1], t_interp, ref_state_msg);

      // spirit_utils::interpGRFArray(local_plan_msg->grfs[i],
      //   local_plan_msg->grfs[i+1], t_interp, grf_array_msg);

      // ref_state_msg = local_plan_msg->states[i];
      grf_array_msg = local_plan_msg->grfs[i];

      break;
    }
  }

  // Declare plan and state data as Eigen vectors
  Eigen::VectorXd ref_body_state(12), grf_array(3 * num_feet_),
      ref_foot_positions(3 * num_feet_), ref_foot_velocities(3 * num_feet_), ref_foot_acceleration(3 * num_feet_);

  // Load plan and state data from messages
  ref_body_state = spirit_utils::bodyStateMsgToEigen(ref_state_msg.body);
  spirit_utils::multiFootStateMsgToEigen(
      ref_state_msg.feet, ref_foot_positions, ref_foot_velocities, ref_foot_acceleration);
  grf_array = spirit_utils::grfArrayMsgToEigen(grf_array_msg);

  // Load contact mode
  std::vector<int> contact_mode(num_feet_);
  for (int i = 0; i < num_feet_; i++) {
    contact_mode[i] = ref_state_msg.feet.feet[i].contact;
  }
  
  // Compute joint torques
  quadKD_->compInvDyn(state_positions, state_velocities, ref_foot_acceleration,grf_array,
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
  
}