#include "global_body_planner/global_body_planner.h"

using namespace planning_utils;

GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, discrete_body_plan_topic;
  std::vector<double> start_state_default = {0.0,0.0,0.4,0.0,0.1,0.0,0.0,0.0};
  std::vector<double> goal_state_default = {8.0,0.0,0.4,0.0,0.0,0.0,0.0,0.0};

  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("global_body_planner/update_rate", update_rate_, 1);
  nh.param<int>("global_body_planner/num_calls", num_calls_, 1);
  nh.param<double>("global_body_planner/replan_time_limit", replan_time_limit_, 0.0);
  nh.param<std::string>("global_body_planner/algorithm", algorithm_, "rrt-connect");

  nh.param<std::vector<double> >("global_body_planner/start_state", start_state_, start_state_default);
  nh.param<std::vector<double> >("global_body_planner/goal_state", goal_state_, goal_state_default);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1,&GlobalBodyPlanner::terrainMapCallback, this);
  body_plan_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(body_plan_topic,1);
  discrete_body_plan_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(discrete_body_plan_topic,1);

}

void GlobalBodyPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);
}

void GlobalBodyPlanner::clearPlan() {
  // Clear old solutions
  body_plan_.clear();
  t_plan_.clear();
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vectors_.clear();
  cost_vectors_times_.clear();
}


void GlobalBodyPlanner::callPlanner() {

  // Get the most recent plan parameters and clear the old solutions
  clearPlan();

  // Copy start and goal states and adjust for ground height
  State start_state;
  State goal_state;
  stdVectorToState(start_state_, start_state);
  stdVectorToState(goal_state_, goal_state);
  start_state[2] += terrain_.getGroundHeight(start_state[0], start_state[1]);
  goal_state[2] += terrain_.getGroundHeight(goal_state[0], goal_state[1]);

  // Make sure terminal states are valid
  if (!isValidState(start_state, terrain_, STANCE)) {
    ROS_WARN("Invalid start state, exiting global planner");
    return;
  }
  if (!isValidState(goal_state, terrain_, STANCE)) {
    ROS_WARN("Invalid goal state, exiting global planner");
    return;
  }
  if (start_state == goal_state) {
    ROS_WARN("Identical start and goal states, exiting global planner");
    return;
  }

  // Initialize statistics variables
  double plan_time;
  int success;
  int vertices_generated;
  double time_to_first_solve;
  double path_duration;
  double total_solve_time = 0;
  double total_vertices_generated = 0;
  double total_path_duration = 0;

  // Set up more objects
  cost_vectors_.reserve(num_calls_);
  cost_vectors_times_.reserve(num_calls_);
  RRTClass rrt_obj;
  RRTConnectClass rrt_connect_obj;
  RRTStarConnectClass rrt_star_connect_obj;

  // Loop through num_calls_ planner calls
  for (int i = 0; i<num_calls_; ++i)
  {
    // Clear out previous solutions and initialize new statistics variables
    state_sequence_.clear();
    action_sequence_.clear();
    std::vector<double> cost_vector;
    std::vector<double> cost_vector_times;

    // Call the appropriate planning method (either RRT-Connect or RRT*-Connect)
    if (algorithm_.compare("rrt-connect") == 0){
      rrt_connect_obj.buildRRTConnect(terrain_, start_state, goal_state,state_sequence_,action_sequence_, replan_time_limit_);
      rrt_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);
    } else if (algorithm_.compare("rrt-star-connect") == 0){
      rrt_star_connect_obj.buildRRTStarConnect(terrain_, start_state, goal_state,state_sequence_,action_sequence_, replan_time_limit_);
      rrt_star_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);
    } else {
      throw std::runtime_error("Invalid algorithm specified");
    }

    // Handle the statistical data
    cost_vectors_.push_back(cost_vector);
    cost_vectors_times_.push_back(cost_vector_times);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_duration += path_duration;

    std::cout << "Vertices generated: " << vertices_generated << std::endl;
    std::cout << "Solve time: " << plan_time << std::endl;
    std::cout << "Time to first solve: " << time_to_first_solve << std::endl;
    std::cout << "Path length: " << cost_vector.back() << std::endl;

    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);
  }
    
  // Report averaged statistics if num_calls_ > 1
  if (num_calls_ > 1)
  {
    std::cout << "Average vertices generated: " << total_vertices_generated/num_calls_ << std::endl;
    std::cout << "Average solve time: " << total_solve_time/num_calls_ << std::endl;
    std::cout << "Average path duration: " << total_path_duration/num_calls_ << std::endl;
  }

  // Interpolate to get full body plan
  double dt = 0.05;
  std::vector<int> interp_phase;
  getInterpPath(state_sequence_, action_sequence_,dt,body_plan_, t_plan_, interp_phase);
}

void GlobalBodyPlanner::addBodyStateToMsg(double t, State body_state, 
    spirit_msgs::BodyPlan& msg) {

  // Make sure the timestamps match the trajectory timing
  ros::Duration time_elapsed(t);
  ros::Time current_time = msg.header.stamp + time_elapsed;

  // Represent each state as an Odometry message
  nav_msgs::Odometry state;
  state.header.frame_id = map_frame_;
  state.header.stamp = current_time;
  state.child_frame_id = "dummy";

  // Transform from RPY to quat msg
  tf2::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(0,body_state[6], atan2(body_state[4],body_state[3]));
  quat_msg = tf2::toMsg(quat_tf);

  // Load the data into the message
  state.pose.pose.position.x = body_state[0];
  state.pose.pose.position.y = body_state[1];
  state.pose.pose.position.z = body_state[2];
  state.pose.pose.orientation = quat_msg;

  state.twist.twist.linear.x = body_state[3];
  state.twist.twist.linear.y = body_state[4];
  state.twist.twist.linear.z = body_state[5];
  state.twist.twist.angular.x = 0;
  state.twist.twist.angular.y = body_state[6];
  state.twist.twist.angular.z = 0;

  msg.states.push_back(state);
}

void GlobalBodyPlanner::publishPlan() {
  // Construct BodyPlan messages
  spirit_msgs::BodyPlan body_plan_msg;
  spirit_msgs::BodyPlan discrete_body_plan_msg;

  // Initialize the headers and types
  ros::Time timestamp = ros::Time::now();
  body_plan_msg.header.stamp = timestamp;
  body_plan_msg.header.frame_id = map_frame_;
  discrete_body_plan_msg.header = body_plan_msg.header;

  // Loop through the interpolated body plan and add to message
  for (int i=0;i<body_plan_.size(); ++i)
    addBodyStateToMsg(t_plan_[i], body_plan_[i], body_plan_msg);

  // Loop through the discrete states and add to message
  for (int i = 0; i<state_sequence_.size(); i++)
    addBodyStateToMsg(t_plan_[i], state_sequence_[i], discrete_body_plan_msg);
  
  // Publish both interpolated body plan and discrete states
  body_plan_pub_.publish(body_plan_msg);
  discrete_body_plan_pub_.publish(discrete_body_plan_msg);
}

void GlobalBodyPlanner::waitForMap() {
    // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while((shared_map == nullptr) && ros::ok())
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>("/terrain_map", nh_);
    ros::spinOnce();
  }
}

void GlobalBodyPlanner::spin() {
  ros::Rate r(update_rate_);

  waitForMap();
  
  // Update the plan
  callPlanner();

  while (ros::ok()) {
    
    // If desired, get a new plan
    // updatePlan();

    // Publish the plan and sleep
    publishPlan();
    ros::spinOnce();
    r.sleep();
  }
}