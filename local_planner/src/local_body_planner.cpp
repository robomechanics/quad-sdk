#include "local_planner/local_body_planner.h"

namespace plt = matplotlibcpp;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalBodyPlanner::LocalBodyPlanner(ros::NodeHandle nh) {
  nh.param<double>("local_body_planner/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string robot_state_traj_topic,robot_state_topic,grf_array_topic, control_traj_topic;
  spirit_utils::loadROSParam(nh, "topics/trajectory", robot_state_traj_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh, "topics/control/grfs", grf_array_topic);
  spirit_utils::loadROSParam(nh, "topics/control/trajectory", control_traj_topic);
  spirit_utils::loadROSParam(nh, "map_frame", map_frame_);

  // Load MPC/system parameters
  spirit_utils::loadROSParam(nh, "local_body_planner/horizon_length",N_);
  spirit_utils::loadROSParam(nh, "local_body_planner/update_rate", update_rate_);
  
  double m,Ixx,Iyy,Izz,mu;
  spirit_utils::loadROSParam(nh, "trajectory_publisher/interp_dt",dt_);
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
  quad_mpc_ = std::make_shared<QuadrupedMPC>(N_,Nx_,Nu_);
  quad_mpc_->setTimestep(dt_);
  quad_mpc_->setMassProperties(m,Ib);
  quad_mpc_->update_weights(Q_vec,U_vec);
  quad_mpc_->update_state_bounds(state_lo, state_hi);
  quad_mpc_->update_friction(mu);

  // Convert kinematics
  kinematics_ = std::make_shared<spirit_utils::SpiritKinematics>();

  // Setup pubs and subs
  robot_state_traj_sub_ = nh_.subscribe(robot_state_traj_topic,1,&LocalBodyPlanner::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&LocalBodyPlanner::robotStateCallback,this);
  grf_array_pub_ = nh_.advertise<spirit_msgs::GRFArray>(grf_array_topic,1);
  traj_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(control_traj_topic,1);
  ROS_INFO("Local Body Planner setup, waiting for callbacks");
}

void LocalBodyPlanner::robotPlanCallback(const spirit_msgs::RobotStateTrajectory::ConstPtr& msg) {
  last_plan_msg_ = msg;
}

void LocalBodyPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  cur_state_ = this->state_to_eigen(*msg);
}

Eigen::VectorXd LocalBodyPlanner::state_to_eigen(spirit_msgs::RobotState robot_state, bool zero_vel) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(Nx_);

  // Position
  state(0) = robot_state.body.pose.pose.position.x;
  state(1) = robot_state.body.pose.pose.position.y;
  state(2) = robot_state.body.pose.pose.position.z;

  // Orientation
  tf2::Quaternion quat;
  tf2::convert(robot_state.body.pose.pose.orientation, quat);
  double r,p,y;
  tf2::Matrix3x3 m(quat);
  m.getRPY(r,p,y);
  state(3) = r;
  state(4) = p;
  state(5) = y;

  if (!zero_vel) {
    // Linear Velocity
    state(6) = robot_state.body.twist.twist.linear.x;
    state(7) = robot_state.body.twist.twist.linear.y;
    state(8) = robot_state.body.twist.twist.linear.z;

    // Angular Velocity
    state(9) = robot_state.body.twist.twist.angular.x;
    state(10) = robot_state.body.twist.twist.angular.y;
    state(11) = robot_state.body.twist.twist.angular.z;
  }

  return state;
}

void LocalBodyPlanner::extractMPCTrajectory(int start_idx,
                                         std::vector<std::vector<bool>> &contact_sequences,
                                         Eigen::MatrixXd &foot_positions, 
                                         Eigen::MatrixXd &ref_traj) {
  int plan_length = last_plan_msg_->states.size();
  contact_sequences.resize(N_);

  foot_positions = Eigen::MatrixXd::Zero(N_,12);
  ref_traj = Eigen::MatrixXd::Zero(Nx_,N_+1);
  ref_traj.col(0) = cur_state_;

  spirit_msgs::RobotState robot_state;
  Eigen::Vector3d foot_pos_body;
  sensor_msgs::JointState joint_state;

  int plan_index;
  bool zero_vel = false;
  for (int i = 0; i < N_; ++i) {

    // Saturate at last state in plane and zero velocity
    if (start_idx+i < plan_length) {
      plan_index = start_idx+i;
    }
    else {
      plan_index = plan_length-1;
      zero_vel = true;
    }

    // Collect state at correct index
    robot_state = last_plan_msg_->states.at(plan_index);

    // Load contact sequence and foot positions
    contact_sequences.at(i).resize(num_legs_);
    std::vector<double> joint_states = robot_state.joints.position;

    for (int leg_idx = 0; leg_idx < num_legs_; ++leg_idx) {
      contact_sequences.at(i).at(leg_idx) = robot_state.feet.feet.at(leg_idx).contact;

      Eigen::Vector3d joint_pos;
      for (int joint_idx = 0; joint_idx < num_joints_per_leg_; ++joint_idx){
        joint_pos(joint_idx) = joint_states.at(leg_idx*num_joints_per_leg_+joint_idx);
      }

      kinematics_->bodyToFootFK(leg_idx, joint_pos, foot_pos_body);
      
      // This should be replaced with a block operation but for now it'll do
      foot_positions(i,leg_idx*num_joints_per_leg_+0) = foot_pos_body(0);
      foot_positions(i,leg_idx*num_joints_per_leg_+1) = foot_pos_body(1); 
      foot_positions(i,leg_idx*num_joints_per_leg_+2) = foot_pos_body(2); 
    }
    // Load state into reference trajectory (w/ zero velocity if we're at end of plan)
    ref_traj.col(i+1) = this->state_to_eigen(robot_state, zero_vel);
  }
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