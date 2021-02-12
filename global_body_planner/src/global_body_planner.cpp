#include "global_body_planner/global_body_planner.h"

using namespace planning_utils;


GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, discrete_body_plan_topic;
  std::vector<double> start_state_default = 
    {0.0,0.0,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::vector<double> goal_state_default = 
    {8.0,0.0,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  nh.param<std::string>("topics/terrain_map", terrain_map_topic_, "/terrain_map");
  nh.param<std::string>("topics/state/ground_truth", robot_state_topic_, "/state/ground_truth");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");
  nh.param<std::string>("map_frame",map_frame_,"map");
  nh.param<double>("global_body_planner/update_rate", update_rate_, 1);
  nh.param<int>("global_body_planner/num_calls", num_calls_, 1);
  nh.param<double>("global_body_planner/max_time", max_time_, 5.0);
  nh.param<double>("global_body_planner/committed_horizon", committed_horizon_, 0);
  nh.param<double>("global_body_planner/state_error_threshold", state_error_threshold_, 0.5);

  nh.param<std::vector<double> >("global_body_planner/start_state", start_state_, start_state_default);
  nh.param<std::vector<double> >("global_body_planner/goal_state", goal_state_, goal_state_default);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic_,1,&GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic_,1,&GlobalBodyPlanner::robotStateCallback, this);
  body_plan_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(body_plan_topic,1);
  discrete_body_plan_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(discrete_body_plan_topic,1);

  // Initialize the current path cost to infinity to ensure the first solution is stored
  current_cost_ = INFTY;
}

void GlobalBodyPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);
}

void GlobalBodyPlanner::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {

  // Quick check to make sure message data has been populated
  if (msg->body.pose.pose.orientation.w > 1e-4) {
    // Get RPY from the state message
    tf2::Quaternion q(
          msg->body.pose.pose.orientation.x,
          msg->body.pose.pose.orientation.y,
          msg->body.pose.pose.orientation.z,
          msg->body.pose.pose.orientation.w);
    q.normalize();
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Only need the states that will be used in planning (ignore roll and yaw)
    robot_state_.clear();
    robot_state_.push_back(msg->body.pose.pose.position.x);
    robot_state_.push_back(msg->body.pose.pose.position.y);
    robot_state_.push_back(msg->body.pose.pose.position.z);
    robot_state_.push_back(roll);
    robot_state_.push_back(pitch);
    robot_state_.push_back(yaw);
    robot_state_.push_back(msg->body.twist.twist.linear.x);
    robot_state_.push_back(msg->body.twist.twist.linear.y);
    robot_state_.push_back(msg->body.twist.twist.linear.z);
    robot_state_.push_back(msg->body.twist.twist.angular.x);
    robot_state_.push_back(msg->body.twist.twist.angular.y);
    robot_state_.push_back(msg->body.twist.twist.angular.z);
  } else {
    ROS_WARN_THROTTLE(0.1, "Invalid quaternion received in GlobalBodyPlanner, exiting callback");
  }
}

bool GlobalBodyPlanner::replanTrigger() {

  if (body_plan_.empty()) {
    return true;
  }

  double state_error_threshold = 0.5;
  std::vector<double> current_state_in_plan_ = body_plan_.front();
  if (poseDistance(robot_state_, current_state_in_plan_) > state_error_threshold_) {
    return true;
  } else {
    return false;
  }
}


int GlobalBodyPlanner::initPlanner() {

  int start_index = 0;

  if (replanTrigger()) {
    start_state_ = robot_state_;
    replan_start_time_ = 0;
    current_cost_ = INFTY;
  } else {
    // Loop through t_plan_ to find the next state after the committed horizon, set as start state
    int N = t_plan_.size();
    for (int i = 0; i < N; i++) {
      if (t_plan_[i] >= committed_horizon_) {

        start_state_.clear();
        start_state_ = body_plan_[i];
        replan_start_time_ = t_plan_[i];
        start_index = i;

        break;
      }
    }
  }
  return start_index;
}

void GlobalBodyPlanner::callPlanner() {

  // Clear out old statistics
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vector_.clear();
  cost_vector_times_.clear();

  // Get the most recent plan parameters
  int start_index = initPlanner();

  // Copy start and goal states and adjust for ground height
  State start_state = fullStateToState(start_state_);
  State goal_state = fullStateToState(goal_state_);
  
  // Make sure terminal states are valid
  if (!isValidState(start_state, terrain_, STANCE)) {
    ROS_WARN_THROTTLE(0.5, "Invalid start state, exiting global planner");
    return;
  }
  if (!isValidState(goal_state, terrain_, STANCE)) {
    ROS_WARN_THROTTLE(0.5, "Invalid goal state, attempting to add in the ground height");
    goal_state[2] += terrain_.getGroundHeight(goal_state_[0], goal_state_[1]);
    if (!isValidState(goal_state, terrain_, STANCE)) {
      ROS_WARN_THROTTLE(0.5, "Invalid goal state, exiting global planner");
      return;
    }
  }
  if (start_state == goal_state) {
    ROS_WARN_THROTTLE(0.5, "Identical start and goal states, exiting global planner");
    return;
  }

  // Initialize statistics variables
  double plan_time;
  int vertices_generated;
  double path_length;
  double path_duration;
  double total_solve_time = 0;
  double total_vertices_generated = 0;
  double total_path_length = 0;
  double total_path_duration = 0;

  // Set up more objects
  RRTConnectClass rrt_connect_obj;

  // Loop through num_calls_ planner calls
  for (int i = 0; i<num_calls_; ++i)
  {
    // Clear out previous solutions and initialize new statistics variables
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    // Call the appropriate planning method (can do if else on algorithm_)
    rrt_connect_obj.runRRTConnect(terrain_, start_state, goal_state,state_sequence,action_sequence, max_time_);
    rrt_connect_obj.getStatistics(plan_time, vertices_generated, path_length, path_duration);

    // Handle the statistical data
    cost_vector_.push_back(path_length);
    cost_vector_times_.push_back(plan_time);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_length += path_length;
    total_path_duration += path_duration;

    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);

    if (path_length < current_cost_) {
      state_sequence_ = state_sequence;
      action_sequence_ = action_sequence;
      current_cost_ = path_length;

      std::cout << "Solve time: " << plan_time << " s" << std::endl;
      std::cout << "Vertices generated: " << vertices_generated << std::endl;
      std::cout << "Path length: " << path_length << " m" << std::endl;
      std::cout << "Path duration: " << path_duration << " s" << std::endl;
      std::cout << std::endl;

      // Clear out old plan and interpolate to get full body plan
      t_plan_.erase(t_plan_.begin()+start_index, t_plan_.end());
      body_plan_.erase(body_plan_.begin()+start_index, body_plan_.end());
      wrench_plan_.erase(wrench_plan_.begin()+start_index, wrench_plan_.end());

      double dt = 0.1;
      std::vector<int> interp_phase;
      getInterpPath(state_sequence_, action_sequence_, dt, replan_start_time_, body_plan_, wrench_plan_, t_plan_, interp_phase);
      
      if (start_index == 0) {
        plan_timestamp_ = ros::Time::now();
      }
    }

    if (!ros::ok()) {
      return;
    }
  }
    
  // Report averaged statistics if num_calls_ > 1
  if (num_calls_ > 1)
  {
    std::cout << "Average vertices generated: " << total_vertices_generated/num_calls_ <<  std::endl;
    std::cout << "Average solve time: " << total_solve_time/num_calls_ << " s" << std::endl;
    std::cout << "Average path length: " << total_path_length/num_calls_ << " s" << std::endl;
    std::cout << "Average path duration: " << total_path_duration/num_calls_ << " s" << std::endl;
    std::cout << std::endl;
  }

}

void GlobalBodyPlanner::addStateWrenchToMsg(double t, FullState body_state, Wrench wrench,
    spirit_msgs::BodyPlan& msg) {

  ROS_ASSERT(body_state.size()==12);

  // Represent each state as an Odometry message
  nav_msgs::Odometry state;
  state.header.frame_id = map_frame_;
  state.header.stamp = msg.header.stamp + ros::Duration(t);
  state.child_frame_id = "dummy";

  // Transform from RPY to quat msg
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(body_state[3],body_state[4],body_state[5]);
  quat_msg = tf2::toMsg(quat_tf);

  // std::cout << "original pitch = " << body_state[7] << std::endl;

  // Load the data into the message
  state.pose.pose.position.x = body_state[0];
  state.pose.pose.position.y = body_state[1];
  state.pose.pose.position.z = body_state[2];
  state.pose.pose.orientation = quat_msg;

  state.twist.twist.linear.x = body_state[6];
  state.twist.twist.linear.y = body_state[7];
  state.twist.twist.linear.z = body_state[8];
  state.twist.twist.angular.x = body_state[9];
  state.twist.twist.angular.y = body_state[10];
  state.twist.twist.angular.z = body_state[11];

  geometry_msgs::Wrench wrench_msg;
  wrench_msg.force.x = wrench[0];
  wrench_msg.force.y = wrench[1];
  wrench_msg.force.z = wrench[2];
  wrench_msg.torque.x = wrench[3];
  wrench_msg.torque.y = wrench[4];
  wrench_msg.torque.z = wrench[5];

  msg.states.push_back(state);
  msg.wrenches.push_back(wrench_msg);
}

void GlobalBodyPlanner::publishPlan() {
  if (body_plan_.empty())
    return;

  // Construct BodyPlan messages
  spirit_msgs::BodyPlan body_plan_msg;
  spirit_msgs::BodyPlan discrete_body_plan_msg;

  // Initialize the headers and types
  body_plan_msg.header.stamp = plan_timestamp_;
  body_plan_msg.header.frame_id = map_frame_;
  discrete_body_plan_msg.header = body_plan_msg.header;

  ROS_ASSERT(t_plan_.front() == 0);

  // Loop through the interpolated body plan and add to message
  for (int i=0;i<body_plan_.size(); ++i)
    addStateWrenchToMsg(t_plan_[i], body_plan_[i], wrench_plan_[i], body_plan_msg);

  // Loop through the discrete states and add to message
  for (int i = 0; i<state_sequence_.size(); i++)
  {
    // Discrete states don't need roll or yaw data, set to zero
    FullState full_discrete_state = stateToFullState(state_sequence_[i],0,0,0,0);
    addStateWrenchToMsg(t_plan_[i], full_discrete_state, wrench_plan_[i], discrete_body_plan_msg);
  }
  
  if (body_plan_msg.states.size() != body_plan_msg.wrenches.size()) {
    throw std::runtime_error("Mismatch between number of states and wrenches, something is wrong");
  }

  // Publish both interpolated body plan and discrete states
  body_plan_pub_.publish(body_plan_msg);
  discrete_body_plan_pub_.publish(discrete_body_plan_msg);
}

void GlobalBodyPlanner::waitForData() {
    // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while((shared_map == nullptr) && ros::ok())
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<spirit_msgs::RobotState const> shared_robot_state;
  while((shared_robot_state == nullptr) && ros::ok())
  {
    shared_robot_state = ros::topic::waitForMessage<spirit_msgs::RobotState>(robot_state_topic_, nh_);
    ros::spinOnce();
  }
}

void GlobalBodyPlanner::spin() {

  plan_from_robot_state_flag_ = true;

  ros::Rate r(update_rate_);

  waitForData();

  while (ros::ok()) {
    
    // Update the plan
    callPlanner();

    // Publish the current best plan and sleep
    publishPlan();
    ros::spinOnce();
    r.sleep();
  }
}