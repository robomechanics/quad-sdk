#include "quad_utils/trajectory_publisher.h"

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
  nh.getParam("trajectory_publisher/traj_name_list_", traj_name_list_);
  nh.param<double>("trajectory_publisher/pause_delay", pause_delay_, 1.0);
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

  import_traj_ = false;
  update_flag_ = true;

  start_time_ = ros::Time::now();
}

std::vector<std::vector<double> > TrajectoryPublisher::loadCSV(std::string filename) {
  
  std::vector<std::vector<double> > data;
  std::ifstream inputFile(filename);
  int l = 0;

  while (inputFile) {
      l++;
      std::string s;
      if (!getline(inputFile, s)) break;
      if (s[0] != '#') {
          std::istringstream ss(s);
          std::vector<double> record;

          while (ss) {
              std::string line;
              if (!getline(ss, line, ','))
                  break;
              try {
                  record.push_back(stod(line));
              }
              catch (const std::invalid_argument e) {
                  std::cout << "NaN found in file " << filename << " line " << l
                       << std::endl;
                  e.what();
              }
          }

          data.push_back(record);
      }
  }

  if (!inputFile.eof()) {
      std::cerr << "Could not read file " << filename << "\n";
      std::__throw_invalid_argument("File not found.");
  }

  return data;
}

void TrajectoryPublisher::importTrajectory() {

  // Clear current trajectory message
  traj_msg_.states.clear();
  traj_msg_.header.frame_id = map_frame_;
  traj_msg_.header.stamp = ros::Time::now();

  // Load the desired values into traj_msg;
  std::string package_path = ros::package::getPath("spirit_utils");
  std::vector<std::vector<double> > trajectory_data_;
  std::vector<std::vector<double> > temp_trajectory_data_;

  for (int i = 0; i < traj_name_list_.size(); i++) {

    std::cout << traj_name_list_.at(i) << std::endl;
// x = m.M[15]
// y = m.M[16]
// z = m.M[17]
// X = m.M[18]
// Y = m.M[19]
// Z = m.M[20]
    if (i == 0) {
      trajectory_data_ = loadCSV(package_path + "/data/TrajectoryData/" + traj_name_list_.at(i) + ".csv");
    } else {

      temp_trajectory_data_ = loadCSV(package_path + "/data/TrajectoryData/" + traj_name_list_.at(i) + ".csv");

      for (int j = 0; j < trajectory_data_.size(); j++) {

        if (j == 1) {
          for (int k = 0; k < temp_trajectory_data_.at(1).size(); k++) {
            temp_trajectory_data_.at(j).at(k) = temp_trajectory_data_.at(j).at(k) + trajectory_data_.at(j).back() + pause_delay_;
          }
        } else if (j == 15 || j == 16 || j == 20) {
          for (int k = 0; k < temp_trajectory_data_.at(1).size(); k++) {
            temp_trajectory_data_.at(j).at(k) = temp_trajectory_data_.at(j).at(k) + trajectory_data_.at(j).back();
          }
        }
        trajectory_data_.at(j).insert(trajectory_data_.at(j).end(), temp_trajectory_data_.at(j).begin(), temp_trajectory_data_.at(j).end());
      }
    }

  }

  t_traj_ = trajectory_data_.at(1);

  // Add states to the traj message
  for (int i = 0; i < t_traj_.size(); i++) {

    // Declare new state message and set timestamp localized to first message
    spirit_msgs::RobotState state;
    state.header.frame_id = map_frame_;
    state.header.stamp = traj_msg_.header.stamp + ros::Duration(t_traj_[i]);

    state.full_trajectory = true;

    tf2::Quaternion q2;
    q2.setRPY(trajectory_data_.at(18).at(i), trajectory_data_.at(19).at(i), trajectory_data_.at(20).at(i));
    q2.normalize();

    state.body.pose.pose.position.x = trajectory_data_.at(15).at(i);
    state.body.pose.pose.position.y = trajectory_data_.at(16).at(i);
    state.body.pose.pose.position.z = trajectory_data_.at(17).at(i);
    state.body.pose.pose.orientation.w = q2.w();
    state.body.pose.pose.orientation.x = q2.x();
    state.body.pose.pose.orientation.y = q2.y();
    state.body.pose.pose.orientation.z = q2.z();
    state.body.twist.twist.linear.x = trajectory_data_.at(33).at(i);
    state.body.twist.twist.linear.y = trajectory_data_.at(34).at(i);
    state.body.twist.twist.linear.z = trajectory_data_.at(35).at(i);
    state.body.twist.twist.angular.x = trajectory_data_.at(36).at(i);
    state.body.twist.twist.angular.y = trajectory_data_.at(37).at(i);
    state.body.twist.twist.angular.z = trajectory_data_.at(38).at(i);

    state.joints.name = {"8", "0", "1", "9","2","3","10","4","5","11","6","7"};

    int num_joints = 12;
    for (int j = 0; j<num_joints; j++) {
      state.joints.position.push_back(trajectory_data_.at(3 + j).at(i));
      state.joints.velocity.push_back(trajectory_data_.at(21 + j).at(i));
      state.joints.effort.push_back(trajectory_data_.at(57 + j).at(i));
    }

    // Add this state to the message
    traj_msg_.states.push_back(state);

  }

  import_traj_ = true;
  update_flag_ = true;

}

void TrajectoryPublisher::robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr& msg) {

  // Save the most recent body plan
  body_plan_msg_ = (*msg);
}

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
  // ros::Duration t_duration = ros::Time::now() - body_plan_msg_.header.stamp;

  ros::Duration t_duration = ros::Time::now() - start_time_;

  // Mod by trajectory duration
  double t = playback_speed_*t_duration.toSec();
  double t_mod = fmod(t, t_traj_.back()); 
  
  // Interpolate to get the correct state and publish it
  quad_msgs::RobotState interp_state = quad_utils::interpRobotStateTraj(traj_msg_,t);
  // quad_msgs::RobotState interp_state = quad_utils::interpRobotStateTraj(traj_msg_,t_mod);

  // If importing trajectory, need to set flag
  if (traj_source_.compare("import")==0) {
    if (t_duration.toSec() <= t_traj_.back()) {
      interp_state.full_trajectory = true;
    }
    // interp_state.full_trajectory = true;
  }

  trajectory_state_pub_.publish(interp_state);

}


void TrajectoryPublisher::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    if (ros::Time::now().toSec() < 1e-9) 
      continue;

    // Update the trajectory and publish
    if (import_traj_ == false) { 
      if (traj_source_.compare("import")==0) {
        importTrajectory();
      }
      start_time_ = ros::Time::now();
    } else if (update_flag_) {
      updateTrajectory();
      publishTrajectory();
    }

    publishTrajectoryState();

    // Collect new messages on subscriber topics
    ros::spinOnce();
    
    // Enforce update rate
    r.sleep();
  }
}
