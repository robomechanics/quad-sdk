#include "twist_body_planner/twist_body_planner.h"

TwistBodyPlanner::TwistBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string robot_state_topic, body_plan_topic, cmd_vel_topic;

  nh_.param<std::string>("topics/state/ground_truth", robot_state_topic,
                         "/state/ground_truth");
  nh_.param<std::string>("topics/global_plan", body_plan_topic, "/body_plan");
  nh_.param<std::string>("topics/cmd_vel", cmd_vel_topic, "/cmd_vel");
  nh_.param<std::string>("map_frame", map_frame_, "map");
  nh_.param<double>("twist_body_planner/cmd_vel_scale", cmd_vel_scale_, 1);
  nh_.param<double>("twist_body_planner/update_rate", update_rate_, 5);
  nh_.param<double>("twist_body_planner/horizon_length", horizon_length_, 1.5);
  nh_.param<double>("twist_body_planner/last_cmd_vel_msg_time_max",
                    last_cmd_vel_msg_time_max_, 1.0);
  quad_utils::loadROSParam(nh_, "local_planner/timestep", dt_);

  // Setup pubs and subs
  cmd_vel_sub_ =
      nh_.subscribe(cmd_vel_topic, 1, &TwistBodyPlanner::cmdVelCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic, 1,
                                   &TwistBodyPlanner::robotStateCallback, this);
  body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic, 1);

  start_state_.resize(12);
  cmd_vel_.resize(6);

  // Zero the velocity to start
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
  plan_timestamp_ = ros::Time::now();
}

void TwistBodyPlanner::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  if ((cmd_vel_[0] != msg->linear.x) || (cmd_vel_[1] != msg->linear.y) ||
      (cmd_vel_[5] != msg->angular.z)) {
    plan_timestamp_ = ros::Time::now();
  }
  // Ignore non-planar components of desired twist
  cmd_vel_[0] = cmd_vel_scale_ * msg->linear.x;
  cmd_vel_[1] = cmd_vel_scale_ * msg->linear.y;
  cmd_vel_[2] = 0;
  cmd_vel_[3] = 0;
  cmd_vel_[4] = 0;
  cmd_vel_[5] = cmd_vel_scale_ * msg->angular.z;

  // Record when this was last reached for safety
  last_cmd_vel_msg_time_ = ros::Time::now();
}

void TwistBodyPlanner::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  tf2::Quaternion q(msg->body.pose.orientation.x, msg->body.pose.orientation.y,
                    msg->body.pose.orientation.z, msg->body.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  start_state_[0] = msg->body.pose.position.x;
  start_state_[1] = msg->body.pose.position.y;
  start_state_[2] = z_des_;
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
  if (start_state_[2] != z_des_) {
    return;
  }

  // Get the most recent plan parameters and clear the old solutions
  clearPlan();

  // Check that we have recent twist data, otherwise set cmd_vel to zero
  ros::Duration time_elapsed_since_msg =
      ros::Time::now() - last_cmd_vel_msg_time_;
  if (time_elapsed_since_msg.toSec() > last_cmd_vel_msg_time_max_) {
    std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0);
    ROS_WARN_THROTTLE(
        1.0, "No cmd_vel data, setting cmd_vel to zero in twist body planner");
  }

  // Integrate to get full body plan
  body_plan_.push_back(start_state_);
  t_plan_.push_back(0);
  for (double t = dt_; t <= horizon_length_; t += dt_) {
    State current_state;
    current_state.resize(12);
    Twist current_cmd_vel = cmd_vel_;

    double yaw = body_plan_.back()[5];
    current_cmd_vel[0] = cmd_vel_[0] * cos(yaw) - cmd_vel_[1] * sin(yaw);
    current_cmd_vel[1] = cmd_vel_[0] * sin(yaw) + cmd_vel_[1] * cos(yaw);

    for (int i = 0; i < 6; i++) {
      current_state[i] = body_plan_.back()[i] + current_cmd_vel[i] * dt_;
      current_state[i + 6] = (current_cmd_vel[i]);
    }

    t_plan_.push_back(t);
    body_plan_.push_back(current_state);
  }
}

void TwistBodyPlanner::addStateWrenchToMsg(double t, int plan_index,
                                           State body_state,
                                           quad_msgs::RobotPlan& msg) {
  // Make sure the timestamps match the trajectory timing
  ros::Time current_time = msg.header.stamp + ros::Duration(t);

  // Represent each state as an Odometry message
  quad_msgs::RobotState state;
  quad_utils::updateStateHeaders(state, msg.header.stamp + ros::Duration(t),
                                 map_frame_, plan_index);

  // Transform from RPY to quat msg
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(body_state[3], body_state[4], body_state[5]);
  quat_msg = tf2::toMsg(quat_tf);

  // Load the data into the message
  state.body.pose.position.x = body_state[0];
  state.body.pose.position.y = body_state[1];
  state.body.pose.position.z = body_state[2];
  state.body.pose.orientation = quat_msg;

  state.body.twist.linear.x = body_state[6];
  state.body.twist.linear.y = body_state[7];
  state.body.twist.linear.z = body_state[8];
  state.body.twist.angular.x = body_state[9];
  state.body.twist.angular.y = body_state[10];
  state.body.twist.angular.z = body_state[11];

  double m = 11.5;
  double g = 9.81;
  quad_msgs::GRFArray grf_msg;
  geometry_msgs::Vector3 vector_msg;
  vector_msg.x = 0;
  vector_msg.y = 0;
  vector_msg.z = m * g;
  geometry_msgs::Point point_msg;
  point_msg.x = body_state[0];
  point_msg.y = body_state[1];
  point_msg.z = body_state[2];

  grf_msg.header = state.header;
  grf_msg.vectors.push_back(vector_msg);
  grf_msg.points.push_back(point_msg);
  bool contact_state = true;
  grf_msg.contact_states.push_back(contact_state);

  int primitive_id = 2;  // Corresponds to a walking primitive
  msg.states.push_back(state);
  msg.grfs.push_back(grf_msg);
  msg.plan_indices.push_back(plan_index);
  msg.primitive_ids.push_back(primitive_id);
}

void TwistBodyPlanner::publishPlan() {
  if (body_plan_.empty()) {
    return;
  }

  // Construct BodyPlan messages
  quad_msgs::RobotPlan robot_plan_msg;

  // plan_timestamp_ = ros::Time::now();

  // Initialize the headers and types
  robot_plan_msg.header.stamp = plan_timestamp_;
  robot_plan_msg.header.frame_id = map_frame_;
  robot_plan_msg.global_plan_timestamp = plan_timestamp_;

  // Loop through the interpolated body plan and add to message
  for (int i = 0; i < body_plan_.size(); ++i) {
    addStateWrenchToMsg(t_plan_[i], i, body_plan_[i], robot_plan_msg);
    // printf("x = %5.3f, dx = %5.3f\n", body_plan_[i][0], body_plan_[i][6]);
  }

  if (robot_plan_msg.states.size() != robot_plan_msg.grfs.size()) {
    throw std::runtime_error(
        "Mismatch between number of states and wrenches, something is wrong");
  }

  // Publish  interpolated body plan
  body_plan_pub_.publish(robot_plan_msg);
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
