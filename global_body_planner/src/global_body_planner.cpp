#include "global_body_planner/global_body_planner.h"

using namespace planning_utils;


GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, discrete_body_plan_topic, body_plan_tree_topic, goal_state_topic;

  quad_utils::loadROSParam(nh_, "topics/terrain_map", terrain_map_topic_);
  quad_utils::loadROSParam(nh_, "topics/state/ground_truth", robot_state_topic_);
  quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_discrete", discrete_body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_tree", body_plan_tree_topic);
  quad_utils::loadROSParam(nh_, "topics/goal_state", goal_state_topic);
  quad_utils::loadROSParam(nh_, "map_frame",map_frame_);
  quad_utils::loadROSParam(nh_, "global_body_planner/update_rate", update_rate_);
  quad_utils::loadROSParam(nh_, "global_body_planner/num_calls", num_calls_);
  quad_utils::loadROSParam(nh_, "global_body_planner/max_planning_time", max_planning_time_);
  quad_utils::loadROSParam(nh_, "global_body_planner/state_error_threshold", state_error_threshold_);
  quad_utils::loadROSParam(nh_, "global_body_planner/startup_delay", startup_delay_);
  quad_utils::loadROSParam(nh_, "global_body_planner/replanning", replanning_allowed_);
  quad_utils::loadROSParam(nh_, "local_planner/timestep", dt_);
  quad_utils::loadROSParam(nh_, "global_body_planner/start_state", start_state_);
  quad_utils::loadROSParam(nh_, "global_body_planner/goal_state", goal_state_);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic_,1,&GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic_,1,&GlobalBodyPlanner::robotStateCallback, this);
  goal_state_sub_ = nh_.subscribe(goal_state_topic,1,&GlobalBodyPlanner::goalStateCallback, this);
  body_plan_pub_ = nh_.advertise<spirit_msgs::RobotPlan>(body_plan_topic,1);
  discrete_body_plan_pub_ = nh_.advertise<spirit_msgs::RobotPlan>(discrete_body_plan_topic,1);
  tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(body_plan_tree_topic,1);

  // Initialize the current path cost to infinity to ensure the first solution is stored
  current_cost_ = INFTY;
  robot_state_ = start_state_;

  quad_utils::loadROSParam(nh,"global_body_planner/H_MAX", planner_config_.H_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/H_MIN", planner_config_.H_MIN);
  quad_utils::loadROSParam(nh,"global_body_planner/H_NOM", planner_config_.H_NOM);
  quad_utils::loadROSParam(nh,"global_body_planner/V_MAX", planner_config_.V_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/V_NOM", planner_config_.V_NOM);
  quad_utils::loadROSParam(nh,"global_body_planner/DY_MAX", planner_config_.DY_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/ROBOT_L", planner_config_.ROBOT_L);
  quad_utils::loadROSParam(nh,"global_body_planner/ROBOT_W", planner_config_.ROBOT_W);
  quad_utils::loadROSParam(nh,"global_body_planner/ROBOT_W", planner_config_.ROBOT_W);
  quad_utils::loadROSParam(nh,"global_body_planner/M_CONST", planner_config_.M_CONST);
  quad_utils::loadROSParam(nh,"global_body_planner/J_CONST", planner_config_.J_CONST);
  quad_utils::loadROSParam(nh,"global_body_planner/G_CONST", planner_config_.G_CONST);
  quad_utils::loadROSParam(nh,"global_body_planner/F_MIN", planner_config_.F_MIN);
  quad_utils::loadROSParam(nh,"global_body_planner/F_MAX", planner_config_.F_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/PEAK_GRF_MIN", planner_config_.PEAK_GRF_MIN);
  quad_utils::loadROSParam(nh,"global_body_planner/PEAK_GRF_MAX", planner_config_.PEAK_GRF_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/MU", planner_config_.MU);
  quad_utils::loadROSParam(nh,"global_body_planner/T_S_MIN", planner_config_.T_S_MIN);
  quad_utils::loadROSParam(nh,"global_body_planner/T_S_MAX", planner_config_.T_S_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/T_F_MIN", planner_config_.T_F_MIN);
  quad_utils::loadROSParam(nh,"global_body_planner/T_F_MAX", planner_config_.T_F_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/KINEMATICS_RES", planner_config_.KINEMATICS_RES);
  quad_utils::loadROSParam(nh,"global_body_planner/BACKUP_TIME", planner_config_.BACKUP_TIME);
  quad_utils::loadROSParam(nh,"global_body_planner/BACKUP_RATIO", planner_config_.BACKUP_RATIO);
  quad_utils::loadROSParam(nh,"global_body_planner/NUM_GEN_STATES", planner_config_.NUM_GEN_STATES);
  quad_utils::loadROSParam(nh,"global_body_planner/GOAL_BOUNDS", planner_config_.GOAL_BOUNDS);

  // If replanning is prohibited, set committed horizon to zero ()
  // if (replanning_allowed_ == false && max_planning_time_ > 0) {
  //   max_planning_time_ = 0;
  //   ROS_INFO("Replanning is prohibited, setting committed horizon to zero");
  // }

}

void GlobalBodyPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);
}

void GlobalBodyPlanner::robotStateCallback(const quad_msgs::RobotState::ConstPtr& msg) {

  // Quick check to make sure message data has been populated and is valid
  geometry_msgs::Quaternion quat = msg->body.pose.orientation;
  if (abs(sqrt(pow(quat.w,2) + pow(quat.x,2) + pow(quat.y,2) + pow(quat.z,2)) - 1.0) < 1e-3) {
    // Get RPY from the state message
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    q.normalize();
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Only need the states that will be used in planning (ignore roll and yaw)
    robot_state_.clear();
    robot_state_.push_back(msg->body.pose.position.x);
    robot_state_.push_back(msg->body.pose.position.y);
    robot_state_.push_back(msg->body.pose.position.z);
    robot_state_.push_back(roll);
    robot_state_.push_back(pitch);
    robot_state_.push_back(yaw);
    robot_state_.push_back(msg->body.twist.linear.x);
    robot_state_.push_back(msg->body.twist.linear.y);
    robot_state_.push_back(msg->body.twist.linear.z);
    robot_state_.push_back(msg->body.twist.angular.x);
    robot_state_.push_back(msg->body.twist.angular.y);
    robot_state_.push_back(msg->body.twist.angular.z);

  } else {
    ROS_WARN_THROTTLE(1.0, "Invalid quaternion received in GlobalBodyPlanner, "
      "returning");
  }
  
}

void GlobalBodyPlanner::goalStateCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

  if (goal_state_msg_ != NULL) {
    if (goal_state_msg_->header.stamp == msg->header.stamp) {
      return;
    }
  }

  goal_state_msg_ = msg;

  goal_state_.resize(12);
  std::fill(goal_state_.begin(), goal_state_.end(), 0);

  goal_state_[0] = goal_state_msg_->point.x;
  goal_state_[1] = goal_state_msg_->point.y;
  goal_state_[2] = 0.3 + planner_config_.terrain.getGroundHeight(
    goal_state_msg_->point.x, goal_state_msg_->point.y);

  // Reset cost so the next amended plan will be accepted
  current_cost_ = INFTY;

  // If the old plan has been executed, allow full replanning
  if (t_plan_.back() <= (ros::Time::now() - plan_timestamp_).toSec()) {
    restart_flag_ = true;
  }
}

void GlobalBodyPlanner::updateRestartFlag() {

  if (body_plan_.empty()) {
    restart_flag_ = true;
    return;
  }

  std::vector<double> current_state_in_plan_ = body_plan_.front();
  if (poseDistance(robot_state_, current_state_in_plan_) > state_error_threshold_) {
    restart_flag_ = true;
  }
}


int GlobalBodyPlanner::initPlanner() {

  int start_index = 0;

  updateRestartFlag();

  if (restart_flag_) {
    start_state_ = robot_state_;
    replan_start_time_ = 0;
    current_cost_ = INFTY;
    length_plan_.clear();
    length_plan_.push_back(0.0);
    restart_flag_ = false;
    ROS_INFO_THROTTLE(1,"GBP restarting from current robot state");
  } else {
    // Loop through t_plan_ to find the next state after the committed horizon, set as start state
    double current_time = (ros::Time::now() - plan_timestamp_).toSec();
    int N = t_plan_.size();
    for (int i = 0; i < N; i++) {
      if (t_plan_.back() <= (current_time + max_planning_time_)) {
        int STOP = -1;
        return STOP;
      }
      if (t_plan_[i] >= (current_time + max_planning_time_)) {

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

bool GlobalBodyPlanner::callPlanner() {

  // Clear out old statistics
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vector_.clear();
  cost_vector_times_.clear();

  // Get the most recent plan parameters
  int start_index = initPlanner();
  if (start_index < 0)
    return false;

  // Copy start and goal states and adjust for ground height
  State start_state = fullStateToState(start_state_);
  State goal_state = fullStateToState(goal_state_);
  
  // Make sure terminal states are valid
  if (!isValidState(start_state, planner_config_, LEAP_STANCE)) {
    ROS_WARN_THROTTLE(2, "Invalid start state, exiting global planner");
    return false;
  }
  goal_state[2] += planner_config_.terrain.getGroundHeight(goal_state_[0], goal_state_[1]);
  if (!isValidState(goal_state, planner_config_, LEAP_STANCE)) {
    ROS_WARN_THROTTLE(2, "Invalid goal state, exiting global planner");
    return false;
  }
  if (start_state == goal_state) {
    ROS_WARN_THROTTLE(2, "Identical start and goal states, exiting global planner");
    return false;
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
    // Exit if ros is down
    if (!ros::ok()) {
      return false;
    }

    // Clear out previous solutions and initialize new statistics variables
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    // Call the appropriate planning method (can do if else on algorithm_)
    bool success = rrt_connect_obj.runRRTConnect(planner_config_, start_state, goal_state,
      state_sequence,action_sequence, max_planning_time_, tree_pub_);
    if (!success) {
      if (num_calls_ > 1) {
        continue;
      } else {
        return false;
      }
    }
    rrt_connect_obj.getStatistics(plan_time, vertices_generated, path_length, path_duration);

    // Add the existing path length to the new
    path_length += length_plan_.at(start_index);

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

      // Clear out old plan and interpolate to get full body plan
      t_plan_.erase(t_plan_.begin()+start_index, t_plan_.end());
      body_plan_.erase(body_plan_.begin()+start_index, body_plan_.end());
      grf_plan_.erase(grf_plan_.begin()+start_index, grf_plan_.end());
      primitive_id_plan_.erase(primitive_id_plan_.begin()+start_index, 
        primitive_id_plan_.end());
      length_plan_.erase(length_plan_.begin()+start_index+1, length_plan_.end());

      std::cout << "Solve time: " << plan_time << " s" << std::endl;
      std::cout << "Vertices generated: " << vertices_generated << std::endl;
      std::cout << "Path length: " << path_length << " m" << std::endl;
      std::cout << "Path duration: " << path_duration << " s" << std::endl;
      std::cout << std::endl;

      getInterpPlan(start_state_, state_sequence_, action_sequence_, dt_, replan_start_time_, 
        body_plan_, grf_plan_, t_plan_, primitive_id_plan_, length_plan_, planner_config_);

      if (body_plan_.size() != length_plan_.size()) {
        ROS_WARN("Mismatched body plan and length");
      }

      if (start_index == 0) {
        plan_timestamp_ = ros::Time::now();
      }
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

void GlobalBodyPlanner::addStateAndGRFToMsg(double t, int plan_index, FullState body_state, 
  GRF grf, int primitive_id, quad_msgs::RobotPlan& msg) {

  ROS_ASSERT(body_state.size()==12);

  // Represent each state as an Odometry message
  quad_msgs::RobotState state;
  quad_utils::updateStateHeaders(state, msg.header.stamp+ros::Duration(t), map_frame_,plan_index);

  // Transform from RPY to quat msg
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(body_state[3],body_state[4],body_state[5]);
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

  quad_msgs::GRFArray grf_msg;
  geometry_msgs::Vector3 vector_msg;
  vector_msg.x = grf[0];
  vector_msg.y = grf[1];
  vector_msg.z = grf[2];
  geometry_msgs::Point point_msg;
  point_msg.x = body_state[0];
  point_msg.y = body_state[1];
  point_msg.z = body_state[2];

  grf_msg.header = state.header;
  grf_msg.vectors.push_back(vector_msg);
  grf_msg.points.push_back(point_msg);

  bool contact_state = (primitive_id != FLIGHT);
  grf_msg.contact_states.push_back(contact_state);

  msg.states.push_back(state);
  msg.grfs.push_back(grf_msg);
  msg.plan_indices.push_back(plan_index);
  msg.primitive_ids.push_back(primitive_id);
}

void GlobalBodyPlanner::publishPlan() {
  if (body_plan_.empty())
    return;

  // std::cout << "body_plan_" << std::endl;
  // for (int i = 0; i < body_plan_.size(); i++) {
  //   std::cout << "t = " << t_plan_[i] << ", state = ";
  //   printState(fullStateToState(body_plan_[i]));
  //   std::cout << ", id = " << primitive_id_plan_[i] << std::endl;
  // }

  // printStateSequence(state_sequence_);
  // printActionSequence(action_sequence_);

  // throw std::runtime_error("Stop");

  // Construct BodyPlan messages
  quad_msgs::RobotPlan robot_plan_msg;
  quad_msgs::RobotPlan discrete_robot_plan_msg;

  // Initialize the headers and types
  robot_plan_msg.header.stamp = plan_timestamp_;
  robot_plan_msg.header.frame_id = map_frame_;
  discrete_robot_plan_msg.header = robot_plan_msg.header;
  robot_plan_msg.global_plan_timestamp = plan_timestamp_;
  discrete_robot_plan_msg.global_plan_timestamp = plan_timestamp_;

  ROS_ASSERT(t_plan_.front() == 0);

  // Loop through the interpolated body plan and add to message
  for (int i=0;i<body_plan_.size(); ++i) {
    addStateAndGRFToMsg(t_plan_[i], i, body_plan_[i], grf_plan_[i],
      primitive_id_plan_[i], robot_plan_msg);
  }

  // Loop through the discrete states and add to message
  for (int i = 0; i<state_sequence_.size(); i++)
  {
    // Discrete states don't need roll, yaw, or timing data, set to zero
    FullState full_discrete_state = stateToFullState(state_sequence_[i],0,0,0,0,0,0);
    addStateAndGRFToMsg(0.0, 0, full_discrete_state, grf_plan_[i], 
      primitive_id_plan_[i], discrete_robot_plan_msg);
  }
  
  if (robot_plan_msg.states.size() != robot_plan_msg.grfs.size()) {
    throw std::runtime_error("Mismatch between number of states and wrenches, something is wrong");
  }

  // Publish both interpolated body plan and discrete states
  body_plan_pub_.publish(robot_plan_msg);
  discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

}

void GlobalBodyPlanner::waitForData() {
    // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while((shared_map == nullptr) && ros::ok())
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<quad_msgs::RobotState const> shared_robot_state;
  while((shared_robot_state == nullptr) && ros::ok())
  {
    shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(robot_state_topic_, nh_);
    ros::spinOnce();
  }
  ROS_INFO("GBP Has state and map information");
}

void GlobalBodyPlanner::getInitialPlan() {

  // Keep track of when the planner started
  ros::Time start_time = ros::Time::now();

  bool success = false;

  // Repeatedly call the planner until the startup delay has elapsed
  while (ros::ok() && ((ros::Time::now() - start_time) < ros::Duration(startup_delay_))) {

    success = callPlanner();
  }  
}

void GlobalBodyPlanner::spin() {

  ros::Rate r(update_rate_);

  // Once we get map and state data, start planning until startup delay is up
  waitForData();
  getInitialPlan();

  // Set the timestamp for the initial plan to now and publish
  plan_timestamp_ = ros::Time::now();
  publishPlan();
  
  // Enter main spin
  while (ros::ok()) {

    // IF allowed, continue updating and publishing the plan
    if (replanning_allowed_) {
      callPlanner();

      if (current_cost_ < INFTY)
        publishPlan();
    }

    // Process callbacks and sleep
    ros::spinOnce();
    r.sleep();
  }
}