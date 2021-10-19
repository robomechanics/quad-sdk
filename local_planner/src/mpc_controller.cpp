#include "local_planner/mpc_controller.h"

namespace plt = matplotlibcpp;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

MPCController::MPCController(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string robot_state_traj_topic,robot_state_topic,grf_array_topic, control_traj_topic;
  spirit_utils::loadROSParam(nh, "topics/trajectory", robot_state_traj_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh, "topics/control/grfs", grf_array_topic);
  spirit_utils::loadROSParam(nh, "topics/control/trajectory", control_traj_topic);
  spirit_utils::loadROSParam(nh, "map_frame", map_frame_);

  // Load MPC/system parameters
  spirit_utils::loadROSParam(nh, "mpc_controller/horizon_length",N_);
  spirit_utils::loadROSParam(nh, "mpc_controller/update_rate", update_rate_);
  
  double m,Ixx,Iyy,Izz,mu;
  spirit_utils::loadROSParam(nh, "mpc_controller/timestep",dt_);
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
  quad_mpc_ = std::make_shared<QuadrupedMPC>();
  quad_mpc_->setTimestep(dt_);
  quad_mpc_->setMassProperties(m,Ib);
  quad_mpc_->update_weights(Q_vec,U_vec);
  quad_mpc_->update_state_bounds(state_lo, state_hi);
  quad_mpc_->update_control_bounds(normal_lo_, normal_hi_);
  quad_mpc_->update_friction(mu);

  // Convert kinematics
quadKD_ = std::make_shared<spirit_utils::QuadKD>();

  // Setup pubs and subs
  robot_state_traj_sub_ = nh_.subscribe(robot_state_traj_topic,1,&MPCController::robotPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&MPCController::robotStateCallback,this);
  grf_array_pub_ = nh_.advertise<spirit_msgs::GRFArray>(grf_array_topic,1);
  traj_pub_ = nh_.advertise<spirit_msgs::RobotPlan>(control_traj_topic,1);
  ROS_INFO("MPC Controller setup, waiting for callbacks");
}

void MPCController::robotPlanCallback(const spirit_msgs::RobotStateTrajectory::ConstPtr& msg) {
  last_plan_msg_ = msg;
}

void MPCController::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  cur_state_ = this->state_to_eigen(*msg);
}

Eigen::VectorXd MPCController::state_to_eigen(spirit_msgs::RobotState robot_state, bool zero_vel) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(Nx_);

  // Position
  state(0) = robot_state.body.pose.position.x;
  state(1) = robot_state.body.pose.position.y;
  state(2) = robot_state.body.pose.position.z;

  // Orientation
  tf2::Quaternion quat;
  tf2::convert(robot_state.body.pose.orientation, quat);
  double r,p,y;
  tf2::Matrix3x3 m(quat);
  m.getRPY(r,p,y);
  state(3) = r;
  state(4) = p;
  state(5) = y;

  if (!zero_vel) {
    // Linear Velocity
    state(6) = robot_state.body.twist.linear.x;
    state(7) = robot_state.body.twist.linear.y;
    state(8) = robot_state.body.twist.linear.z;

    // Angular Velocity
    state(9) = robot_state.body.twist.angular.x;
    state(10) = robot_state.body.twist.angular.y;
    state(11) = robot_state.body.twist.angular.z;
  }

  return state;
}

void MPCController::extractMPCTrajectory(int start_idx,
                                         std::vector<std::vector<bool>> &contact_sequences,
                                         Eigen::MatrixXd &foot_positions, 
                                         Eigen::MatrixXd &ref_traj) {
  int plan_length = last_plan_msg_->states.size();
  contact_sequences.resize(N_);

  foot_positions = Eigen::MatrixXd::Zero(N_,12);
  ref_traj = Eigen::MatrixXd::Zero(N_+1,Nx_);
  ref_traj.row(0) = cur_state_;

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

    quadKD_->bodyToFootFKBodyFrame(leg_idx, joint_pos, foot_pos_body);
      
      // This should be replaced with a block operation but for now it'll do
      foot_positions(i,leg_idx*num_joints_per_leg_+0) = foot_pos_body(0);
      foot_positions(i,leg_idx*num_joints_per_leg_+1) = foot_pos_body(1); 
      foot_positions(i,leg_idx*num_joints_per_leg_+2) = foot_pos_body(2); 
    }
    // Load state into reference trajectory (w/ zero velocity if we're at end of plan)
    ref_traj.row(i+1) = this->state_to_eigen(robot_state, zero_vel);
  }
}

void MPCController::publishGRFArray() {
  if (last_plan_msg_ == NULL) {
    ROS_WARN_THROTTLE(0.5, "No robot trajectory plan in MPCController, exiting");
    return;
  }

  if (last_plan_msg_->states.size() == 0) {
    ROS_WARN_THROTTLE(0.5, "Received robot trajectory with no states in MPCController, exiting");
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

    msg.vectors[i].x = control_traj(0,num_joints_per_leg_*i);
    msg.vectors[i].y = control_traj(0,num_joints_per_leg_*i+1);
    msg.vectors[i].z = control_traj(0,num_joints_per_leg_*i+2);
  }
  
  msg.header.stamp = ros::Time::now();
  grf_array_pub_.publish(msg);
}

void MPCController::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    ROS_WARN_THROTTLE(5, "MPC node still operating");

    ros::spinOnce();
    
    // Publish control input data
    publishGRFArray();
    
    r.sleep();
  }
}