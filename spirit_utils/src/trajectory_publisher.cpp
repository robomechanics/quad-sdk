#include "spirit_utils/trajectory_publisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, foot_plan_continuous_topic, 
    trajectory_topic, trajectory_state_topic, ground_truth_state_topic;

  nh.param<std::string>("topics/global_plan", 
    body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/foot_plan_continuous", 
    foot_plan_continuous_topic, "/foot_plan_continuous");

  nh.param<std::string>("topics/state/trajectory", 
    trajectory_state_topic, "/state/trajectory");
  nh.param<std::string>("topics/trajectory", 
    trajectory_topic, "/trajectory");
    nh.param<std::string>("topics/state/ground_truth", 
    ground_truth_state_topic, "/state/ground_truth");

  nh.param<std::string>("map_frame",map_frame_,"map");
  nh.param<std::string>("trajectory_publisher/traj_source", traj_source_, "topic");
  nh.param<double>("trajectory_publisher/update_rate", update_rate_, 30);
  nh.param<double>("trajectory_publisher/interp_dt", interp_dt_, 0.05);
  nh.param<double>("trajectory_publisher/playback_speed", playback_speed_, 1.0);

  // Setup subs and pubs
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,
    &TrajectoryPublisher::robotPlanCallback, this);
  multi_foot_plan_continuous_sub_ = nh_.subscribe(foot_plan_continuous_topic,1,
    &TrajectoryPublisher::multiFootPlanContinuousCallback, this);
  ground_truth_state_sub_ = nh_.subscribe(ground_truth_state_topic,1,
    &TrajectoryPublisher::robotStateCallback, this);

  trajectory_state_pub_ = nh_.advertise<spirit_msgs::RobotState>
    (trajectory_state_topic,1);
  trajectory_pub_ = nh_.advertise<spirit_msgs::RobotStateTrajectory>
    (trajectory_topic,1);

}

void TrajectoryPublisher::importTrajectory() {
  // For Mike

  // Clear current trajectory message
  traj_msg_.states.clear();
  traj_msg_.header.frame_id = map_frame_;
  traj_msg_.header.stamp = ros::Time::now();

  // Load the desired balues into traj_msg;

}

void TrajectoryPublisher::robotPlanCallback(const spirit_msgs::RobotPlan::ConstPtr& msg) {

  // Save the most recent body plan
  body_plan_msg_ = (*msg);
}

void TrajectoryPublisher::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  // Save the most recent robot state
  robot_state_msg_ = msg;
}

void TrajectoryPublisher::multiFootPlanContinuousCallback(const 
  spirit_msgs::MultiFootPlanContinuous::ConstPtr& msg) {

  if (msg->header.stamp != multi_foot_plan_continuous_msg_.header.stamp) {
    // Save the most recent foot plan
    multi_foot_plan_continuous_msg_ = (*msg);
    update_flag_ = true;
  } 

}

void TrajectoryPublisher::updateTrajectory() {
  // spirit_utils::FunctionTimer timer(__FUNCTION__);

  // Make sure we have data for both the body and the foot plan
  if (body_plan_msg_.states.empty() || multi_foot_plan_continuous_msg_.states.empty())
    return;

  // Make sure the foot plan is no longer than the body plan
  ros::Duration t_body_plan_ros = body_plan_msg_.states.back().header.stamp - 
    body_plan_msg_.states.front().header.stamp;
  double t_body_plan = t_body_plan_ros.toSec();

  ros::Duration t_foot_plan_ros = multi_foot_plan_continuous_msg_.states.back().header.stamp - 
    multi_foot_plan_continuous_msg_.states.front().header.stamp;
  double t_foot_plan = t_foot_plan_ros.toSec();

  if (t_body_plan < t_foot_plan) {
    ROS_DEBUG_THROTTLE(1, "Foot plan duration is longer than body plan, traj prublisher "
      "will wait");
    return;
  }

  // Create t_traj vector with specified dt
  double traj_duration = std::min(t_body_plan, t_foot_plan);
  double t = 0;
  t_traj_.clear();
  while (t < traj_duration) {
    t_traj_.push_back(t);
    t += interp_dt_;
  }

  // Declare robot state trajectory message
  traj_msg_.states.clear();
  traj_msg_.header.frame_id = map_frame_;
  traj_msg_.header.stamp = body_plan_msg_.header.stamp;

  spirit_utils::SpiritKinematics kinematics;

  // Add states to the traj message
  for (int i = 0; i < t_traj_.size(); i++) {

    // Declare new state message and set timestamp localized to first message
    spirit_msgs::RobotState state;
    state.header.frame_id = map_frame_;
    state.header.stamp = traj_msg_.header.stamp + ros::Duration(t_traj_[i]);

    // Interpolate body and foot plan
    int primitive_id;
    spirit_msgs::GRFArray grf_array;
    spirit_utils::interpRobotPlan(body_plan_msg_,t_traj_[i], state,primitive_id, grf_array);
    state.feet = spirit_utils::interpMultiFootPlanContinuous(
      multi_foot_plan_continuous_msg_,t_traj_[i]);

    // Compute joint data with IK
    spirit_utils::ikRobotState(kinematics, state.body, state.feet, state.joints);

    // Add this state to the message
    traj_msg_.states.push_back(state);

  }

  // timer.report();
}

void TrajectoryPublisher::publishTrajectory() {
  trajectory_pub_.publish(traj_msg_);
}

void TrajectoryPublisher::publishTrajectoryState() {

  // Wait until we actually have data
  if (traj_msg_.states.empty()) {
    if (robot_state_msg_ != NULL) {
      trajectory_state_pub_.publish(*robot_state_msg_);
    }
    return;
  }
    
  // Get the current duration since the beginning of the plan
  ros::Duration t_duration = ros::Time::now() - body_plan_msg_.header.stamp;

  // Mod by trajectory duration
  double t = playback_speed_*t_duration.toSec();
  double t_mod = fmod(t, t_traj_.back());
  
  // Interpolate to get the correct state and publish it
  spirit_msgs::RobotState interp_state = spirit_utils::interpRobotStateTraj(traj_msg_,t);
  // spirit_msgs::RobotState interp_state = spirit_utils::interpRobotStateTraj(traj_msg_,t_mod);
  trajectory_state_pub_.publish(interp_state);
}


void TrajectoryPublisher::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Update the trajectory and publish
    if (traj_source_.compare("import")==0) {
      importTrajectory();
    } else if (update_flag_) {
      updateTrajectory();
      publishTrajectory();
      update_flag_ = false;
    }

    publishTrajectoryState();

    // Collect new messages on subscriber topics
    ros::spinOnce();
    
    // Enforce update rate
    r.sleep();
  }
}
