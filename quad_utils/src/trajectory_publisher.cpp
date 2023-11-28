#include "quad_utils/trajectory_publisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
<<<<<<< HEAD
  std::string body_plan_topic, trajectory_state_topic;

  nh.param<std::string>("topics/global_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/state/trajectory", trajectory_state_topic,
                        "/state/trajectory");

  nh.param<std::string>("map_frame", map_frame_, "map");
  nh.param<std::string>("trajectory_publisher/traj_source", traj_source_,
                        "topic");
  nh.param<double>("trajectory_publisher/update_rate", update_rate_, 30);

  // Setup subs and pubs
  body_plan_sub_ = nh_.subscribe(body_plan_topic, 1,
                                 &TrajectoryPublisher::robotPlanCallback, this);

  trajectory_state_pub_ =
      nh_.advertise<quad_msgs::RobotState>(trajectory_state_topic, 1);

  // Initialize kinematics object
  quadKD_ = std::make_shared<quad_utils::QuadKD>();
}

void TrajectoryPublisher::importTrajectory() {
  // Load the desired values into body_plan_msg_ here
  return;
}

void TrajectoryPublisher::robotPlanCallback(
    const quad_msgs::RobotPlan::ConstPtr& msg) {
=======
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

  trajectory_state_pub_ = nh_.advertise<quad_msgs::RobotState>
    (trajectory_state_topic,1);
  trajectory_pub_ = nh_.advertise<quad_msgs::RobotStateTrajectory>
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

void TrajectoryPublisher::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg) {

>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
  // Save the most recent body plan
  body_plan_msg_ = (*msg);
}

<<<<<<< HEAD
void TrajectoryPublisher::publishTrajectoryState() {
  // Wait until we actually have data
  if (body_plan_msg_.states.empty()) {
    return;
  }

  // Get the current time in the trajectory since the beginning of the plan
  double traj_duration = (body_plan_msg_.states.back().header.stamp -
                          body_plan_msg_.states.front().header.stamp)
                             .toSec();
  double t =
      (ros::Time::now() - body_plan_msg_.states.front().header.stamp).toSec();

  // Ensure the trajectory remains valid
  t = std::min(t, traj_duration);

  // Interpolate to get the correct state and publish it
  quad_msgs::RobotState interp_state;
  int interp_primitive_id;
  quad_msgs::GRFArray interp_grf;

  quad_utils::interpRobotPlan(body_plan_msg_, t, interp_state,
                              interp_primitive_id, interp_grf);

  // Fill joints and feet with dummy data
  if (interp_state.joints.name.empty()) {
    interp_state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                                "10", "4", "5", "11", "6", "7"};
    interp_state.joints.position = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    interp_state.joints.velocity = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    interp_state.joints.effort = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  }
  quad_utils::fkRobotState(*quadKD_, interp_state.body, interp_state.joints,
                           interp_state.feet);

  trajectory_state_pub_.publish(interp_state);
}

void TrajectoryPublisher::spin() {
  ros::Rate r(update_rate_);
  if (traj_source_.compare("import") == 0) {
    importTrajectory();
  }
  while (ros::ok()) {
    // Publish the trajectory state
=======
void TrajectoryPublisher::robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {

  // Save the most recent robot state
  robot_state_msg_ = msg;
}

void TrajectoryPublisher::multiFootPlanContinuousCallback(const 
  quad_msgs::MultiFootPlanContinuous::ConstPtr& msg) {

  if (msg->header.stamp != multi_foot_plan_continuous_msg_.header.stamp && 
    (msg->states.front().traj_index == 0) ) {
    // Save the most recent foot plan
    multi_foot_plan_continuous_msg_ = (*msg);
    update_flag_ = true;
  } 

}

void TrajectoryPublisher::updateTrajectory() {
  // quad_utils::FunctionTimer timer(__FUNCTION__);

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
  // double traj_duration = std::min(t_body_plan, t_foot_plan);
  double traj_duration = t_body_plan;
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

  quad_utils::QuadKD kinematics;
  

  // Add states to the traj message
  for (int i = 0; i < t_traj_.size(); i++) {

    // Declare new state message and set timestamp localized to first message
    quad_msgs::RobotState state;
    state.header.frame_id = map_frame_;
    state.header.stamp = traj_msg_.header.stamp + ros::Duration(t_traj_[i]);

    // Interpolate body and foot plan
    int primitive_id;
    quad_msgs::GRFArray grf_array;
    quad_utils::interpRobotPlan(body_plan_msg_,t_traj_[i], state,primitive_id, grf_array);

    // If we have foot data then load that, otherwise just set joints to zero
    if (t_traj_[i] < t_foot_plan) {
      state.feet = quad_utils::interpMultiFootPlanContinuous(
        multi_foot_plan_continuous_msg_,t_traj_[i]);

      // Compute joint data with IK
      quad_utils::ikRobotState(kinematics, state.body, state.feet, state.joints);
    } else {
      state.joints.name = {"8","0","1","9","2","3","10","4","5","11","6","7"};
      state.joints.position = {0,0,0,0,0,0,0,0,0,0,0,0};
      state.joints.velocity = {0,0,0,0,0,0,0,0,0,0,0,0};
      state.joints.effort = {0,0,0,0,0,0,0,0,0,0,0,0};
      quad_utils::fkRobotState(kinematics, state.body, state.joints, state.feet);
    }

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
      // trajectory_state_pub_.publish(*robot_state_msg_);
    }
    return;
  }
    
  // Get the current duration since the beginning of the plan
  ros::Duration t_duration = ros::Time::now() - body_plan_msg_.header.stamp;

  // Mod by trajectory duration
  double t = playback_speed_*t_duration.toSec();
  double t_mod = fmod(t, t_traj_.back());

  // Interpolate to get the correct state and publish it
  // std::cout << traj_msg_ << std::endl;
  quad_msgs::RobotState interp_state = quad_utils::interpRobotStateTraj(traj_msg_,t);
  // quad_msgs::RobotState interp_state = quad_utils::interpRobotStateTraj(traj_msg_,t_mod);
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
    }

>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
    publishTrajectoryState();

    // Collect new messages on subscriber topics
    ros::spinOnce();
<<<<<<< HEAD

=======
    
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
    // Enforce update rate
    r.sleep();
  }
}
