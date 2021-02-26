#include "mpc_controller/mpc_controller.h"

MPCController::MPCController(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string robot_state_traj_topic, grf_array_topic,foot_plan_discrete_topic,body_plan_topic, discrete_body_plan_topic;
  spirit_utils::loadROSParam(nh, "topics/trajectory", robot_state_traj_topic);
  spirit_utils::loadROSParam(nh, "topics/control/grfs", grf_array_topic);
  spirit_utils::loadROSParam(nh, "map_frame", map_frame_);

  spirit_utils::loadROSParamDefault(nh, "mpc_controller/update_rate", update_rate_, 50.0);

  // Load MPC/system parameters
  spirit_utils::loadROSParam(nh, "mpc_controller/horizon_length",N_);
  
  double m,Ixx,Iyy,Izz,mu;
  spirit_utils::loadROSParam(nh, "trajectory_publisher/interp_dt",dt_);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_mass",m);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_ixx",Ixx);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_iyy",Iyy);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_izz",Izz);
  spirit_utils::loadROSParam(nh, "mpc_controller/friction_mu",mu);
  spirit_utils::loadROSParam(nh, "mpc_controller/normal_lo",normal_lo_);
  spirit_utils::loadROSParam(nh, "mpc_controller/normal_hi",normal_hi_);
  std::vector<double> state_weights, control_weights, state_lower_bound, state_upper_bound;
  spirit_utils::loadROSParam(nh, "mpc_controller/state_weights",state_weights);
  spirit_utils::loadROSParam(nh, "mpc_controller/control_weights",control_weights);
  spirit_utils::loadROSParam(nh, "mpc_controller/state_lower_bound",state_lower_bound);
  spirit_utils::loadROSParam(nh, "mpc_controller/state_upper_bound",state_upper_bound);

  // Load state weights and bounds
  Eigen::MatrixXd Qx = Eigen::MatrixXd::Zero(Nx_, Nx_);
  Eigen::VectorXd state_lo = Eigen::VectorXd::Zero(Nx_);
  Eigen::VectorXd state_hi = Eigen::VectorXd::Zero(Nx_);
  for (int i = 0; i < Nx_; ++i) {
    Qx(i,i) = state_weights.at(i);
    state_lo(i) = state_lower_bound.at(i);
    state_hi(i) = state_upper_bound.at(i);
  }

  // Load control weights
  Eigen::MatrixXd Ru = Eigen::MatrixXd::Zero(Nu_,Nu_);
  for (int i = 0; i < 3; ++i) { // for each dimension
    for (int j = 0; j < 4; ++j) { //for each leg
      Ru(4*i + j,4*i+j) = control_weights.at(i);
    }
  }
  Ru(Nu_-1,Nu_-1) = 1e-6; //gravity weight term

  std::vector<Eigen::MatrixXd> Q_vec(N_+1);
  std::vector<Eigen::MatrixXd> U_vec(N_);
  for (int i = 0; i < N_+1; ++i) {
    Q_vec.at(i) = Qx;
  }

  for (int i = 0; i < N_; ++i) {
    U_vec.at(i) = Ru;
  }

  // Robot body inertia matrix
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Zero();
  Ib.diagonal() << Ixx,Iyy,Izz;

  // Create convex mpc wrapper class
  quad_mpc_ = std::make_shared<QuadrupedMPC>(N_,Nx_,Nu_);
  quad_mpc_->setTimestep(dt_);
  quad_mpc_->setMassProperties(m,Ib);
  quad_mpc_->update_weights(Q_vec,U_vec);
  quad_mpc_->update_state_bounds(state_lo, state_hi);
  quad_mpc_->update_friction(mu);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  // Setup pubs and subs
  robot_state_traj_sub_ = nh_.subscribe(robot_state_traj_topic,1,&MPCController::robotPlanCallback, this);
  grf_array_pub_ = nh_.advertise<spirit_msgs::GRFArray>(grf_array_topic,1);
  ROS_INFO("MPC Controller setup, waiting for callbacks");
}

void MPCController::robotPlanCallback(const spirit_msgs::RobotStateTrajectory::ConstPtr& msg) {
  last_plan_msg_ = msg;
}

Eigen::VectorXd MPCController::state_to_eigen(spirit_msgs::RobotState robot_state) {

}

void MPCController::extractMPCTrajectory(int start_idx,
                                         std::vector<std::vector<bool>> &contact_sequences,
                                         Eigen::MatrixXd &foot_positions, 
                                         Eigen::MatrixXd &ref_traj) {
  int plan_length = last_plan_msg_->states.size();

  contact_sequences.resize(N_);
  foot_positions = Eigen::MatrixXd::Zero(N_,12);
  ref_traj = Eigen::MatrixXd::Zero(N_+1,Nx_);

  ref_traj.row(0) = this->state_to_eigen(cur_state_);

  spirit_msgs::RobotState robot_state;
  Eigen::Vector3d foot_pos_body;
  sensor_msgs::JointState joint_state;
  
  for (int i = 0; i <= N_; ++i) {
    if (start_idx + i >= plan_length) break;

    // Collect state at correct index
    robot_state = last_plan_msg_->states.at(start_idx+i);

    // Load contact sequence and foot positions
    contact_sequences.at(i).resize(4);
    std::vector<double> joint_states = robot_state.joints.position;
    for (int leg_idx = 0; leg_idx < 4; ++leg_idx) {
      contact_sequences.at(i).at(leg_idx) = robot_state.feet.feet.at(leg_idx).contact;

      Eigen::Vector3d joint_pos;
      for (int joint_idx = 0; joint_idx < 3; ++joint_idx){
        joint_pos(joint_idx) = joint_states.at(leg_idx*4+joint_idx);
      }
      kinematics_->bodyToFootFK(leg_idx, joint_pos, foot_pos_body);
      foot_positions.block(i,leg_idx*4,1,3) = foot_pos_body;

    }

    // Load state into reference trajectory
    ref_traj.row(i+1) = this->state_to_eigen(robot_state); 
  }

  // If we ran off the end of the plan, hold last position, zero out last velocity
  //if end_idx - start_idx < N_ {
    // do what I said to do above
  //}

  //bodyToFootFK(int leg_index, Eigen::Vector3d joint_state, 
  //Eigen::Vector3d &foot_pos_body);
}

void MPCController::publishGRFArray() {
  if (last_plan_msg_ == NULL) {
    ROS_WARN_THROTTLE(0.5, "No robot trajectory plan in MPCController, exiting");
    return;
  }
  double last_plan_age_seconds = spirit_utils::getROSMessageAgeInMs(last_plan_msg_->header)/1000.0;

  //ROS_INFO("Last plan age (s): %f    Dt: %f", last_plan_age_seconds, dt_);
  int start_idx = ceil(last_plan_age_seconds/dt_) + 1;

  // Containers for MPC inputs
  std::vector<std::vector<bool>> contact_sequences;
  Eigen::MatrixXd foot_positions;
  Eigen::MatrixXd ref_traj;
  this->extractMPCTrajectory(start_idx, contact_sequences, foot_positions, ref_traj);

  spirit_msgs::GRFArray msg;

  msg.header.stamp = ros::Time::now();
  grf_array_pub_.publish(msg);
}

void MPCController::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Publish control input data
    publishGRFArray();
    ros::spinOnce();
    r.sleep();
  }
}