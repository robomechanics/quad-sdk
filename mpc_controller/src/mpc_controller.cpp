#include "mpc_controller/mpc_controller.h"

MPCController::MPCController(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string robot_state_traj_topic, grf_array_topic,foot_plan_discrete_topic,body_plan_topic, discrete_body_plan_topic;
  spirit_utils::loadROSParam(nh, "topics/trajectory", robot_state_traj_topic);
  spirit_utils::loadROSParam(nh, "topics/control/grfs", grf_array_topic);
  spirit_utils::loadROSParam(nh, "topics/foot_plan_discrete", foot_plan_discrete_topic);
  spirit_utils::loadROSParam(nh, "topics/body_plan", body_plan_topic);
  spirit_utils::loadROSParam(nh, "topics/discrete_body_plan", discrete_body_plan_topic);
  spirit_utils::loadROSParam(nh, "map_frame", map_frame_);

  spirit_utils::loadROSParamDefault(nh, "mpc_controller/update_rate", update_rate_, 50.0);

  // Load MPC/system parameters
  int N; // MPC Horizon
  spirit_utils::loadROSParam(nh, "mpc_controller/horizon_length",N);
  
  double dt,m,Ixx,Iyy,Izz;
  spirit_utils::loadROSParam(nh, "mpc_controller/timestep",dt);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_mass",m);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_ixx",Ixx);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_iyy",Iyy);
  spirit_utils::loadROSParam(nh, "mpc_controller/body_izz",Izz);
  std::vector<double> state_weights, control_weights, state_lower_bound, state_upper_bound;
  spirit_utils::loadROSParam(nh, "mpc_controller/state_weights",state_weights);
  spirit_utils::loadROSParam(nh, "mpc_controller/control_weights",control_weights);
  spirit_utils::loadROSParam(nh, "mpc_controller/state_lower_bound",state_lower_bound);
  spirit_utils::loadROSParam(nh, "mpc_controller/state_upper_bound",state_upper_bound);

  // Fixed parameters
  const int Nx = 12;
  const int Nu = 13;

  // Load state weights and bounds
  Eigen::MatrixXd Qx = Eigen::MatrixXd::Zero(Nx, Nx);
  Eigen::VectorXd state_lo = Eigen::VectorXd::Zero(Nx);
  Eigen::VectorXd state_hi = Eigen::VectorXd::Zero(Nx);
  for (int i = 0; i < Nx; ++i) {
    Qx(i,i) = state_weights.at(i);
  }

  // Load control weights
  Eigen::MatrixXd Ru = Eigen::MatrixXd::Zero(Nu,Nu);
  for (int i = 0; i < 3; ++i) { // for each dimension
    for (int j = 0; j < 4; ++j) { //for each leg
      Ru(4*i + j,4*i+j) = control_weights.at(i);
    }
  }
  Ru(Nu-1,Nu-1) = 1e-6; //gravity weight

  //std::cout << Qx << std::endl;
  //std::cout << Ru << std::endl;


  // Robot body inertia matrix
  Eigen::Matrix3d Ib = Eigen::Matrix3d::Zero();
  Ib.diagonal() << Ixx,Iyy,Izz;

  // Create qp class
  quad_mpc_ = std::make_shared<QuadrupedMPC>(N,Nx,Nu);
  quad_mpc_->setTimestep(dt);
  quad_mpc_->setMassProperties(m,Ib);


  // Setup pubs and subs
  robot_state_traj_sub_ = nh_.subscribe(robot_state_traj_topic,1,&MPCController::robotPlanCallback, this);
  footstep_plan_sub_ = nh_.subscribe(foot_plan_discrete_topic,1,&MPCController::footPlanDiscreteCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&MPCController::bodyPlanCallback, this);
  discrete_body_plan_sub_ = nh_.subscribe(discrete_body_plan_topic,1,&MPCController::discreteBodyPlanCallback, this);
  grf_array_pub_ = nh_.advertise<spirit_msgs::GRFArray>(grf_array_topic,1);
}

void MPCController::robotPlanCallback(
  const spirit_msgs::RobotStateTrajectory::ConstPtr& msg) {

  last_plan_msg_ = (*msg);
}

void MPCController::footPlanDiscreteCallback(const spirit_msgs::MultiFootPlanDiscrete::ConstPtr& msg) {
  // ROS_INFO("In footPlanDiscreteCallback");
}

void MPCController::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  // ROS_INFO("In bodyPlanCallback");
}

void MPCController::discreteBodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  // ROS_INFO("In discreteBodyPlanCallback");
}

void MPCController::publishGRFArray() {
  // ROS_INFO("In ControlInput");
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