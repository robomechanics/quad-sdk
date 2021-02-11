#include "spirit_utils/trajectory_publisher.h"

TrajectoryPublisher::TrajectoryPublisher(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, foot_plan_continuous_topic, 
    trajectory_topic, trajectory_state_topic;

  nh.param<std::string>("topics/body_plan", 
    body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/foot_plan_continuous", 
    foot_plan_continuous_topic, "/foot_plan_continuous");
  nh.param<std::string>("topics/state/trajectory", 
    trajectory_state_topic, "/state/trajectory");
  nh.param<std::string>("topics/trajectory", 
    trajectory_topic, "/trajectory");

  nh.param<std::string>("map_frame",map_frame_,"map");
  nh.param<double>("trajectory_publisher/update_rate", update_rate_, 30);
  nh.param<double>("trajectory_publisher/interp_dt", interp_dt_, 0.05);
  nh.param<double>("trajectory_publisher/playback_speed", playback_speed_, 1.0);

  // Setup subs and pubs
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,
    &TrajectoryPublisher::bodyPlanCallback, this);
  foot_plan_continuous_sub_ = nh_.subscribe(foot_plan_continuous_topic,1,
    &TrajectoryPublisher::footPlanContinuousCallback, this);

  trajectory_state_pub_ = nh_.advertise<spirit_msgs::RobotState>
    (trajectory_state_topic,1);
  trajectory_pub_ = nh_.advertise<spirit_msgs::RobotStateTrajectory>
    (trajectory_topic,1);
}

void TrajectoryPublisher::importTrajectory() {
  // For Mike

}

void TrajectoryPublisher::bodyPlanCallback(const
  spirit_msgs::BodyPlan::ConstPtr& msg) {

  trajectory_timestamp_ = msg->header.stamp;

  t_body_plan_.clear();
  body_plan_.clear();
  // Loop through the message to get the state info and add to private vector
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    // Convert orientation from quaternion to rpy for interp (To do: slerp)
    tf2::Quaternion q;
    tf2::convert(msg->states[i].pose.pose.orientation,q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get the time associated with this data
    ros::Duration t_plan = msg->states[i].header.stamp - 
      msg->states[0].header.stamp;
    t_body_plan_.push_back(t_plan.toSec());

    // std::cout << "pitch = " << pitch << std::endl;

    // Get the state associated with this data
    std::vector<double> s(12);
    s[0] = msg->states[i].pose.pose.position.x;
    s[1] = msg->states[i].pose.pose.position.y;
    s[2] = msg->states[i].pose.pose.position.z;
    s[3] = roll;
    s[4] = pitch;
    s[5] = yaw;
    s[6] = msg->states[i].twist.twist.linear.x;
    s[7] = msg->states[i].twist.twist.linear.y;
    s[8] = msg->states[i].twist.twist.linear.z;
    s[9] = msg->states[i].twist.twist.angular.x;
    s[10] = msg->states[i].twist.twist.angular.y;
    s[11] = msg->states[i].twist.twist.angular.z;
    body_plan_.push_back(s);
  }
}

void TrajectoryPublisher::footPlanContinuousCallback(const 
  spirit_msgs::MultiFootPlanContinuous::ConstPtr& msg) {

  if (t_body_plan_.empty())
    return;

  spirit_utils::SpiritKinematics spirit;
  t_joints_plan_.clear();
  joints_plan_.clear();

  for (int i = 0; i < msg->states.size(); i++) {

    // Initialize joint state and timing
    std::vector<double> joint_state;
    ros::Duration t_foot_pos = msg->states[i].header.stamp - msg->header.stamp;
    double t = t_foot_pos.toSec();

    for (int j=0; j < msg->states[i].feet.size(); j++) {

      // Get foot position data
      Eigen::Vector3d foot_pos;
      foot_pos[0] = msg->states[i].feet[j].position.x;
      foot_pos[1] = msg->states[i].feet[j].position.y;
      foot_pos[2] = msg->states[i].feet[j].position.z;

      // Get corresponding body plan data
      std::vector<double> body_state = math_utils::interpMat(t_body_plan_, 
        body_plan_, t);
      Eigen::Vector3d body_pos = {body_state[0], body_state[1], body_state[2]};
      Eigen::Vector3d body_rpy = {body_state[3], body_state[4], body_state[5]};

      // Compute IK to get joint data
      Eigen::Vector3d leg_joint_state;
      spirit.legIK(j,body_pos,body_rpy,foot_pos,leg_joint_state);

      // Add to the joint state vector
      joint_state.push_back(leg_joint_state[0]);
      joint_state.push_back(leg_joint_state[1]);
      joint_state.push_back(leg_joint_state[2]);
    }

    t_joints_plan_.push_back(t);
    joints_plan_.push_back(joint_state);
  }
}

void TrajectoryPublisher::updateTrajectory() {

  if (t_body_plan_.empty() || t_joints_plan_.empty())
    return;

  t_traj_.clear();

  // Create t_traj vector with specified dt, make sure it will be in range
  double traj_duration = std::min(t_body_plan_.back(), t_joints_plan_.back());
  double t = 0;
  while (t < traj_duration) {
    t_traj_.push_back(t);
    t += interp_dt_;
  }

  // Double check that the all the time vectors are synchronized
  ROS_ASSERT(t_traj_.front() == 0);
  ROS_ASSERT(t_body_plan_.front() == 0);

  // Declare robot state trajectory message
  traj_msg_.states.clear();
  traj_msg_.header.frame_id = map_frame_;
  traj_msg_.header.stamp = trajectory_timestamp_;

  for (int i = 0; i < t_traj_.size(); i++) {
    
    // Declare new state message and set timestamp localized to first message
    spirit_msgs::RobotState state;
    state.header.frame_id = map_frame_;
    state.header.stamp = traj_msg_.header.stamp + ros::Duration(t_traj_[i]);

    // Interpolate to get the correct body and joints state
    std::vector<double> body_state = math_utils::interpMat(t_body_plan_, 
      body_plan_, t_traj_[i]);
    std::vector<double> joint_state = math_utils::interpMat(t_joints_plan_, 
      joints_plan_, t_traj_[i]);

    // Fill the message with the interpolated data
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(body_state[3],body_state[4],body_state[5]);
    quat_msg = tf2::toMsg(quat_tf);

    state.body.pose.pose.position.x = body_state[0];
    state.body.pose.pose.position.y = body_state[1];
    state.body.pose.pose.position.z = body_state[2];   
    state.body.pose.pose.orientation = quat_msg;

    state.body.twist.twist.linear.x = body_state[6];
    state.body.twist.twist.linear.y = body_state[7];
    state.body.twist.twist.linear.z = body_state[8];
    state.body.twist.twist.angular.x = body_state[9];
    state.body.twist.twist.angular.y = body_state[10];
    state.body.twist.twist.angular.z = body_state[11];

    std::vector<std::string> joint_names = {"8", "0", "1", "9","2", "3", "10",
      "4","5", "11", "6", "7"};
    std::vector<double> joint_vel (12,0);
    std::vector<double> joint_effort (12,0);

    state.joints.name = joint_names;
    state.joints.position = joint_state;
    state.joints.velocity = joint_vel;
    state.joints.effort = joint_effort;

    traj_msg_.states.push_back(state);
  }
}

void TrajectoryPublisher::publishTrajectory() {
  trajectory_pub_.publish(traj_msg_);
}

void TrajectoryPublisher::publishTrajectoryState() {

  if (traj_msg_.states.empty())
    return;

  // Get the current duration since the beginning of the plan
  ros::Duration t_duration = ros::Time::now() - trajectory_timestamp_;

  // Mod by trajectory duration
  double t = fmod(playback_speed_*t_duration.toSec(), t_traj_.back());

  // Loop through all states in the trajectory
  for (int i = 0; i < traj_msg_.states.size()-1; i++) {

    // Get times of current and next state for checking
    ros::Duration t_state = traj_msg_.states[i].header.stamp - 
      traj_msg_.states[0].header.stamp;
    ros::Duration t_state_next = traj_msg_.states[i+1].header.stamp - 
      traj_msg_.states[0].header.stamp;

    // Check times to see if this is the current state
    if (t >= t_state.toSec() && t < t_state_next.toSec()) {

      // Publish current state (to do: interpolate for smoother visualization)
      trajectory_state_pub_.publish(traj_msg_.states[i]);
      return;
    }
  }
}


void TrajectoryPublisher::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    updateTrajectory();
    publishTrajectoryState();

    // Collect new messages on subscriber topics
    ros::spinOnce();
    
    // Enforce update rate
    r.sleep();
  }
}
