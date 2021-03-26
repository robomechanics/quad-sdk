#include "local_planner/local_planner.h"

namespace plt = matplotlibcpp;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

LocalPlanner::LocalPlanner(ros::NodeHandle nh) {
	nh_ = nh;

    // Load rosparams from parameter server
  std::string global_plan_topic,robot_state_topic,local_plan_topic, foot_plan_discrete_topic;
  spirit_utils::loadROSParam(nh, "topics/plan/global", global_plan_topic);
  spirit_utils::loadROSParam(nh_,"topics/state/ground_truth",robot_state_topic);
  spirit_utils::loadROSParam(nh, "topics/plan/local", local_plan_topic);
  spirit_utils::loadROSParam(nh, "topics/plan/footsteps", foot_plan_discrete_topic);
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
  foot_plan_discrete_pub_ = nh_.advertise<spirit_msgs::MultiFootPlanDiscrete>(foot_plan_discrete_topic,1);

  // Initialize nominal footstep positions projected down from the hips
  Eigen::Vector3d nominal_joint_state;
  nominal_joint_state << 0, 0.78, 1.57; // Default stand angles

  for (int i = 0; i < N_; ++i) {
    for (int j = 0; j < num_legs_; ++j) {
      Eigen::Vector3d toe_body_pos;
      kinematics_->bodyToFootFK(j, nominal_joint_state, toe_body_pos);
      hip_projected_foot_positions_.block<1,3>(i,j*3) = toe_body_pos;
    }
  }

  ROS_INFO("LocalPlanner setup complete, waiting for callbacks");
}

void LocalPlanner::globalPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  global_plan_msg_ = msg;
}

void LocalPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  current_state_ = spirit_utils::stateMsgToEigen(*msg);
}

void LocalPlanner::computeLocalPlan() {

  // Initialize continuous foot plan with hip projected locations (constant)
  foot_positions_ = hip_projected_foot_positions_;

  // Iteratively generate body and footstep plans (non-parallelizable)
  for (int i = 0; i < iterations_; i++) {
    
    // Compute body plan with MPC
    local_body_planner_.computePlan(current_state_, global_plan_msg_, foot_positions_,
      contact_sequences_, body_plan_, grf_plan_);
    
    // Compute the new footholds to match that body plan
    local_footstep_planner_.computeDiscretePlan(body_plan_, grf_plan_, foot_plan_discrete_msg_);

    // Convert the footholds to a FootTraj representation for body planning
    local_footstep_planner_.convertDiscretePlanToEigen(foot_plan_discrete_msg_, foot_positions_,
      contact_sequences_);
    
  }

  // Compute the continuous-time foot plan msg for publishing
  local_footstep_planner_.computeContinuousPlan(body_plan_, foot_plan_discrete_msg_,
    foot_plan_continuous_msg_); 

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

    robot_state_msg.body = eigenToOdomMsg(body_plan_.row(i));
    robot_state_msg.feet = foot_plan_continuous_msg_[i];
    math_utils::ikRobotState(robot_state_msg);

    spirit_msgs::GRFArray grf_array_msg;
    grf_array_msg.header.stamp = timestamp;
    grf_array_msg.header.frame = map_frame_;

    grf_array_msg = eigenToGRFArrayMsg(grf_plan_.row(i), foot_plan_continuous_msg_[i]);
    

    local_plan_msg.states.push_back(robot_state_msg);
    local_plan_msg.grfs.push_back(grf_array_msg);
  }

  // Publish
  local_plan_pub_.publish(local_plan_msg);
  foot_plan_discrete_pub_.publish(foot_plan_discrete_msg_);
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