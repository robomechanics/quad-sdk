#include "local_planner/local_body_planner.h"

namespace plt = matplotlibcpp;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalBodyPlanner::LocalBodyPlanner(ros::NodeHandle nh) {
  // Load MPC parameters 
  double m,Ixx,Iyy,Izz,mu;
  spirit_utils::loadROSParam(nh, "local_planner/timestep",dt_);
  spirit_utils::loadROSParam(nh, "local_body_planner/body_mass",m);
  spirit_utils::loadROSParam(nh, "local_body_planner/body_ixx",Ixx);
  spirit_utils::loadROSParam(nh, "local_body_planner/body_iyy",Iyy);
  spirit_utils::loadROSParam(nh, "local_body_planner/body_izz",Izz);
  spirit_utils::loadROSParam(nh, "local_body_planner/friction_mu",mu);
  spirit_utils::loadROSParam(nh, "local_body_planner/normal_lo",normal_lo_);
  spirit_utils::loadROSParam(nh, "local_body_planner/normal_hi",normal_hi_);

  std::vector<double> state_weights, control_weights, state_lower_bound, state_upper_bound;
  spirit_utils::loadROSParam(nh, "local_body_planner/state_weights",state_weights);
  spirit_utils::loadROSParam(nh, "local_body_planner/control_weights",control_weights);
  spirit_utils::loadROSParam(nh, "local_body_planner/state_lower_bound",state_lower_bound);
  spirit_utils::loadROSParam(nh, "local_body_planner/state_upper_bound",state_upper_bound);

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
  quad_mpc_ = std::make_shared<QuadrupedMPC>();
  quad_mpc_->setTimestep(dt_);
  quad_mpc_->setMassProperties(m,Ib);
  quad_mpc_->update_weights(Q_vec,U_vec);
  quad_mpc_->update_state_bounds(state_lo, state_hi);
  quad_mpc_->update_friction(mu);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  // Setup pubs and subs
  ROS_INFO("Local Body Planner setup, waiting for callbacks");
}

void LocalBodyPlanner::publishGRFArray() {
  if (last_plan_msg_ == NULL) {
    ROS_WARN_THROTTLE(0.5, "No robot trajectory plan in LocalBodyPlanner, exiting");
    return;
  }

  if (last_plan_msg_->states.size() == 0) {
    ROS_WARN_THROTTLE(0.5, "Received robot trajectory with no states in LocalBodyPlanner, exiting");
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

  // Pass inputs into solver and solve
  quad_mpc_->update_contact(contact_sequences, normal_lo_, normal_hi_);
  
  //quad_mpc_->update_dynamics(ref_traj,foot_positions);
  quad_mpc_->update_dynamics_hip_projected_feet(ref_traj);//,foot_positions);
  
  Eigen::MatrixXd x_out;
  if (!quad_mpc_->solve(cur_state_, ref_traj, x_out)) {
    std::cout << "Failed solve: " << std::endl;
    std::cout << "Current state: " << std::endl << cur_state_.format(CleanFmt) << std::endl;
    std::cout << "Reference trajectory: " << std::endl << ref_traj.format(CleanFmt) << std::endl;
    std::cout << "Foot Placements in body frame: " << std::endl << foot_positions.format(CleanFmt) << std::endl;
    throw 10;
  }

  double f_val; // currently not getting populated
  Eigen::MatrixXd opt_traj, control_traj;
  quad_mpc_->get_output(x_out, opt_traj, control_traj, f_val);

  // Copy normal forces into GRFArray
  spirit_msgs::GRFArray msg; // control traj nu x n
  msg.points.resize(num_legs_);
  msg.vectors.resize(num_legs_);
  for (int i = 0; i < num_legs_; ++i) {
    msg.points[i].x = foot_positions(0,num_joints_per_leg_*i);
    msg.points[i].y = foot_positions(0,num_joints_per_leg_*i+1);
    msg.points[i].z = foot_positions(0,num_joints_per_leg_*i+2);

    msg.vectors[i].x = control_traj(num_joints_per_leg_*i,0);
    msg.vectors[i].y = control_traj(num_joints_per_leg_*i+1,0);
    msg.vectors[i].z = control_traj(num_joints_per_leg_*i+2,0);
  }
  
  msg.header.stamp = ros::Time::now();
  grf_array_pub_.publish(msg);
}