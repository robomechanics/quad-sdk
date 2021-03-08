#include "twist_body_planner/twist_body_planner.h"

TwistBodyPlanner::TwistBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string robot_state_topic, body_plan_topic, cmd_vel_topic;

  nh.param<std::string>("topics/state/ground_truth", robot_state_topic, "/state/ground_truth");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/cmd_vel", cmd_vel_topic, "/cmd_vel");
  nh.param<std::string>("map_frame", map_frame_,"map");
  nh.param<double>("twist_body_planner/update_rate", update_rate_, 5);
  nh.param<double>("twist_body_planner/horizon_length", horizon_length_, 1.5);
  nh.param<double>("twist_body_planner/last_cmd_vel_msg_time_max",last_cmd_vel_msg_time_max_,1.0);

  // Setup pubs and subs
  cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic,1,&TwistBodyPlanner::cmdVelCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&TwistBodyPlanner::robotStateCallback, this);
  body_plan_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(body_plan_topic,1);

  start_state_.resize(12);
  cmd_vel_.resize(6);

  // Zero the velocity to start
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
}

void TwistBodyPlanner::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {

  if ((cmd_vel_[0] != msg->linear.x) || (cmd_vel_[1] != msg->linear.y) || 
    (cmd_vel_[5] != msg->angular.z)) {
    
    plan_timestamp_ = ros::Time::now();
  }
  // Ignore non-planar components of desired twist
  cmd_vel_[0] = msg->linear.x;
  cmd_vel_[1] = msg->linear.y;
  cmd_vel_[2] = 0;
  cmd_vel_[3] = 0;
  cmd_vel_[4] = 0;
  cmd_vel_[5] = msg->angular.z;

  // Record when this was last reached for safety
  last_cmd_vel_msg_time_ = ros::Time::now();
  
}

void TwistBodyPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  tf2::Quaternion q(
        msg->body.pose.pose.orientation.x,
        msg->body.pose.pose.orientation.y,
        msg->body.pose.pose.orientation.z,
        msg->body.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  start_state_[0] = msg->body.pose.pose.position.x;
  start_state_[1] = msg->body.pose.pose.position.y;
  start_state_[2] = z_des;
  start_state_[3] = 0;
  start_state_[4] = 0;
  start_state_[5] = yaw;
  start_state_[6] = cmd_vel_[0];
  start_state_[7] = cmd_vel_[1];
  start_state_[8] = cmd_vel_[2];
  start_state_[9] = cmd_vel_[3];
  start_state_[10] = cmd_vel_[4];
  start_state_[11] = cmd_vel_[5];
}


void TwistBodyPlanner::clearPlan() {
  // Clear old solutions
  body_plan_.clear();
  t_plan_.clear();
}


void TwistBodyPlanner::updatePlan() {

  if (start_state_[2] != z_des) {
    return;
  }

  // Get the most recent plan parameters and clear the old solutions
  clearPlan();

  // Check that we have recent twist data, otherwise set cmd_vel to zero
  ros::Duration time_elapsed_since_msg = ros::Time::now() - last_cmd_vel_msg_time_;
  if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_) {
    std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
    ROS_WARN_THROTTLE(1.0, "No cmd_vel data, setting cmd_vel to zero in twist body planner");
  }

  // Integrate to get full body plan
  double dt = 0.1;

  body_plan_.push_back(start_state_);
  t_plan_.push_back(0);
  for (double t = dt; t <= horizon_length_; t += dt) {
    State current_state;
    current_state.resize(12);
    Twist current_cmd_vel = cmd_vel_;

    double yaw = body_plan_.back()[5];
    current_cmd_vel[0] = cmd_vel_[0]*cos(yaw) - cmd_vel_[1]*sin(yaw);
    current_cmd_vel[1] = cmd_vel_[0]*sin(yaw) + cmd_vel_[1]*cos(yaw);

    for (int i = 0; i < 6; i ++) {
      current_state[i] = body_plan_.back()[i] + current_cmd_vel[i]*dt;
      current_state[i+6] = (cmd_vel_[i]);
    }

    t_plan_.push_back(t);
    body_plan_.push_back(current_state);
  }
}

void TwistBodyPlanner::addStateWrenchToMsg(double t, State body_state,
    spirit_msgs::BodyPlan& msg) {

  // Make sure the timestamps match the trajectory timing
  ros::Time current_time = msg.header.stamp + ros::Duration(t);

  // Represent each state as an Odometry message
  nav_msgs::Odometry state;
  state.header.frame_id = map_frame_;
  state.header.stamp = current_time;
  state.child_frame_id = "dummy";

  // Transform from RPY to quat msg
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(body_state[3], body_state[4], body_state[5]);
  quat_msg = tf2::toMsg(quat_tf);

  // Load the data into the message
  state.pose.pose.position.x = body_state[0];
  state.pose.pose.position.y = body_state[1];
  state.pose.pose.position.z = body_state[2];
  state.pose.pose.orientation = quat_msg;

  state.twist.twist.linear.x = body_state[6];
  state.twist.twist.linear.y = body_state[7];
  state.twist.twist.linear.z = body_state[9];
  state.twist.twist.angular.x = body_state[9];
  state.twist.twist.angular.y = body_state[10];
  state.twist.twist.angular.z = body_state[11];

  double m = 11.5;
  double g = 9.81;
  spirit_msgs::GRFArray grf_array_msg;
  geometry_msgs::Vector3 vector_msg;
  vector_msg.x = 0;
  vector_msg.y = 0;
  vector_msg.z = m*g;
  geometry_msgs::Point point_msg;
  point_msg.x = body_state[0];
  point_msg.y = body_state[1];
  point_msg.z = body_state[2];

  grf_array_msg.header.stamp = current_time;
  grf_array_msg.vectors.push_back(vector_msg);
  grf_array_msg.points.push_back(point_msg);
  grf_array_msg.contact_states.push_back(true);

  int primitive_id = 2; // Corresponds to a walking primitive
  msg.states.push_back(state);
  msg.grfs.push_back(grf_array_msg);
  msg.primitive_ids.push_back(primitive_id);
}

void TwistBodyPlanner::publishPlan() {

  if (body_plan_.empty()) {
    return;
  }

  // Construct BodyPlan messages
  spirit_msgs::BodyPlan body_plan_msg;

  // Initialize the headers and types
  body_plan_msg.header.stamp = plan_timestamp_;
  body_plan_msg.header.frame_id = map_frame_;

  // Loop through the interpolated body plan and add to message
  for (int i=0;i<body_plan_.size(); ++i)
    addStateWrenchToMsg(t_plan_[i], body_plan_[i], body_plan_msg);
  
  if (body_plan_msg.states.size() != body_plan_msg.grfs.size()) {
    throw std::runtime_error("Mismatch between number of states and wrenches, something is wrong");
  }

  // Publish  interpolated body plan
  body_plan_pub_.publish(body_plan_msg);

}

void TwistBodyPlanner::spin() {
  ros::Rate r(update_rate_);

  plan_timestamp_ = ros::Time::now();

  while (ros::ok()) {
    
    // Update the plan
    updatePlan();

    // Publish the current best plan and sleep
    publishPlan();
    ros::spinOnce();
    r.sleep();
  }
}