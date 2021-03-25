#include "local_planner/local_planner.h"

namespace plt = matplotlibcpp;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string global_plan_topic,robot_state_topic,local_plan_topic;
  spirit_utils::loadROSParam(nh, "topics/plan/global", global_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh, "topics/plan/local", local_plan_topic);
  spirit_utils::loadROSParam(nh, "map_frame", map_frame_);

  // Load system parameters
  spirit_utils::loadROSParam(nh, "local_planner/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh, "local_planner/horizon_length",N_);
  spirit_utils::loadROSParam(nh, "local_planner/timestep",dt_);
  spirit_utils::loadROSParam(nh, "local_planner/iterations",iterations_);

  // Setup pubs and subs
  global_plan_sub_ = nh_.subscribe(global_plan_topic,1,&LocalPlanner::globalPlanCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&LocalPlanner::robotStateCallback,this);
  local_plan_pub_ = nh_.advertise<spirit_msgs::LocalPlan>(local_plan_topic,1);

  multi_foot_plan_discrete_hip_projected_ = stuff;

  ROS_INFO("Local Planner setup complete, waiting for callbacks");
}

void LocalPlanner::globalPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  last_global_plan_msg_ = msg;
}

void LocalPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  current_state_ = this->state_msg_to_eigen(*msg);
}

Eigen::VectorXd LocalPlanner::state_msg_to_eigen(spirit_msgs::RobotState robot_state, bool zero_vel) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(Nx_);

  // Position
  state(0) = robot_state.pose.pose.position.x;
  state(1) = robot_state.pose.pose.position.y;
  state(2) = robot_state.pose.pose.position.z;

  // Orientation
  tf2::Quaternion quat;
  tf2::convert(robot_state.pose.pose.orientation, quat);
  double r,p,y;
  tf2::Matrix3x3 m(quat);
  m.getRPY(r,p,y);
  state(3) = r;
  state(4) = p;
  state(5) = y;

  if (!zero_vel) {
    // Linear Velocity
    state(6) = robot_state.twist.twist.linear.x;
    state(7) = robot_state.twist.twist.linear.y;
    state(8) = robot_state.twist.twist.linear.z;

    // Angular Velocity
    state(9) = robot_state.twist.twist.angular.x;
    state(10) = robot_state.twist.twist.angular.y;
    state(11) = robot_state.twist.twist.angular.z;
  }

  return state;
}

void LocalPlanner::computeLocalPlan() {

  multi_foot_plan_discrete_ = multi_foot_plan_discrete_hip_projected_;

  // Iteratively generate body and footstep plans (non-parallelizable)
  for (int i = 0; i < iterations_; i++) {
    local_body_planner_.computePlan(current_state_, multi_foot_plan_discrete_,
      body_plan_, grf_plan_);
    local_footstep_planner_.computeDiscretePlan(body_plan_, grf_plan_,
      multi_foot_plan_discrete_);
  }

  // Compute continuous foot plan from final footstep plan
  local_footstep_planner_.computeContinuousPlan(body_plan_, multi_foot_plan_discrete_,
    multi_foot_plan_continuous_msg_);

}

void LocalPlanner::publishLocalPlan() {

  // Initialize local plan message, set timestamp to now
  spirit_msgs::LocalPlan local_plan_msg;
  local_plan_msg.header.stamp = ros::Time::now();
  local_plan_msg.header.frame = map_frame_;

  // Add body, foot, joint, and grf data to the local plan message
  for (int i = 0; i < body_plan_; i++) {

    ros::Time timestamp = local_plan_msg.header.stamp + ros::Duration(i*dt_);

    spirit_msgs::RobotState robot_state_msg;
    robot_state_msg.header.stamp = timestamp;
    robot_state_msg.header.frame = map_frame_;

    robot_state_msg.body = eigenToStateMsg(body_plan_[i]);
    robot_state_msg.feet = multi_foot_plan_continuous_msg_[i];
    math_utils::ikRobotState(robot_state_msg);

    spirit_msgs::GRFArray grf_array_msg;
    grf_array_msg.header.stamp = timestamp;
    grf_array_msg.header.frame = map_frame_;

    grf_array_msg = eigenToGRFMsg(grf_plan_[i], multi_foot_plan_continuous_msg_[i]);
    

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
  }

  // Publish
  local_plan_pub_.publish(local_plan_msg);
}

void LocalPlanner::spin() {

  ros::Rate r(compute_rate_);

  while (ros::ok()) {

    ros::spinOnce();
    
    // Publish local plan data (state reference traj and desired GRF array)
    computeLocalPlan();
    publishLocalPlan();
    
    r.sleep();
  }
}