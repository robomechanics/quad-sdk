#include "global_body_planner/global_body_planner.h"

using namespace planning_utils;

GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  std::string body_plan_topic, discrete_body_plan_topic, body_plan_tree_topic,
      goal_state_topic;
  std::vector<double> goal_state_vec(2);

  quad_utils::loadROSParam(nh_, "topics/start_state", robot_state_topic_);
  quad_utils::loadROSParam(nh_, "topics/goal_state", goal_state_topic);
  quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic_);
  quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_discrete",
                           discrete_body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_tree",
                           body_plan_tree_topic);
  quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/update_rate",
                           update_rate_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/num_calls", num_calls_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/max_planning_time",
                           max_planning_time_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/pos_error_threshold",
                           pos_error_threshold_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/startup_delay",
                           reset_publish_delay_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/replanning",
                           replanning_allowed_);
  quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/goal_state",
                           goal_state_vec);
<<<<<<< HEAD

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(
      terrain_map_topic_, 1, &GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(
      robot_state_topic_, 1, &GlobalBodyPlanner::robotStateCallback, this);
  goal_state_sub_ = nh_.subscribe(goal_state_topic, 1,
                                  &GlobalBodyPlanner::goalStateCallback, this);
  body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic, 1);
  discrete_body_plan_pub_ =
      nh_.advertise<quad_msgs::RobotPlan>(discrete_body_plan_topic, 1);
  tree_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(body_plan_tree_topic, 1);

  // Load planner config
  bool enable_leaping;
  planner_config_.loadParamsFromServer(nh);
  nh_.param<bool>("global_body_planner/enable_leaping", enable_leaping, true);
  if (!enable_leaping) {
    planner_config_.enable_leaping = false;
    planner_config_.num_leap_samples = 0;
    planner_config_.h_min = 0;
    planner_config_.h_max = 0.5;
=======

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(
      terrain_map_topic_, 1, &GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(
      robot_state_topic_, 1, &GlobalBodyPlanner::robotStateCallback, this);
  goal_state_sub_ = nh_.subscribe(goal_state_topic, 1,
                                  &GlobalBodyPlanner::goalStateCallback, this);
  body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic, 1);
  discrete_body_plan_pub_ =
      nh_.advertise<quad_msgs::RobotPlan>(discrete_body_plan_topic, 1);
  tree_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(body_plan_tree_topic, 1);

  // Load planner config
  bool enable_leaping;
  planner_config_.loadParamsFromServer(nh);
  nh_.param<bool>("global_body_planner/enable_leaping", enable_leaping, true);
  if (!enable_leaping) {
    planner_config_.enable_leaping = false;
    planner_config_.num_leap_samples = 0;
    planner_config_.h_min = 0;
    planner_config_.h_max = 0.5;
=======
  std::string body_plan_topic, discrete_body_plan_topic, goal_state_topic;
  std::vector<double> start_state_default = 
    {0.0,0.0,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  std::vector<double> goal_state_default = 
    {8.0,0.0,0.3,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  nh.param<std::string>("topics/terrain_map", terrain_map_topic_, "/terrain_map");
  nh.param<std::string>("topics/state/ground_truth", robot_state_topic_, "/state/ground_truth");
  nh.param<std::string>("topics/global_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/global_plan_discrete", discrete_body_plan_topic, "/discrete_body_plan");
  nh.param<std::string>("topics/goal_state", goal_state_topic, "/clicked_point");
  nh.param<std::string>("map_frame",map_frame_,"map");
  nh.param<double>("global_body_planner/update_rate", update_rate_, 1);
  nh.param<int>("global_body_planner/num_calls", num_calls_, 1);
  nh.param<double>("global_body_planner/max_planning_time", max_planning_time_, 5.0);
  nh.param<double>("global_body_planner/state_error_threshold", state_error_threshold_, 0.5);
  nh.param<double>("global_body_planner/startup_delay", startup_delay_, 0.0);
  nh.param<bool>("global_body_planner/replanning", replanning_allowed_, true);
  quad_utils::loadROSParam(nh, "local_planner/timestep", dt_);

  nh.param<std::vector<double> >("global_body_planner/start_state", start_state_, start_state_default);
  nh.param<std::vector<double> >("global_body_planner/goal_state", goal_state_, goal_state_default);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic_,1,&GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(robot_state_topic_,1,&GlobalBodyPlanner::robotStateCallback, this);
  goal_state_sub_ = nh_.subscribe(goal_state_topic,1,&GlobalBodyPlanner::goalStateCallback, this);
  body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic,1);
  discrete_body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(discrete_body_plan_topic,1);

  // Initialize the current path cost to infinity to ensure the first solution is stored
  current_cost_ = INFTY;
  robot_state_ = start_state_;

  quad_utils::loadROSParam(nh,"global_body_planner/H_MAX", planner_config_.H_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/H_MIN", planner_config_.H_MIN);
  quad_utils::loadROSParam(nh,"global_body_planner/V_MAX", planner_config_.V_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/V_NOM", planner_config_.V_NOM);
  quad_utils::loadROSParam(nh,"global_body_planner/DY_MAX", planner_config_.DY_MAX);
  quad_utils::loadROSParam(nh,"global_body_planner/ROBOT_L", planner_config_.ROBOT_L);
  quad_utils::loadROSParam(nh,"global_body_planner/ROBOT_W", planner_config_.ROBOT_W);
  quad_utils::loadROSParam(nh,"global_body_planner/ROBOT_W", planner_config_.ROBOT_W);
  quad_utils::loadROSParam(nh,"global_body_planner/M_CONST", planner_config_.M_CONST);
  quad_utils::loadROSParam(nh,"global_body_planner/J_CONST", planner_config_.J_CONST);
  quad_utils::loadROSParam(nh,"global_body_planner/G_CONST", planner_config_.G_CONST);
  quad_utils::loadROSParam(nh,"global_body_planner/F_MAX", planner_config_.F_MAX);
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
  if (replanning_allowed_ == false && max_planning_time_ > 0) {
    max_planning_time_ = 0;
    ROS_INFO("Replanning is prohibited, setting committed horizon to zero");
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  }

  // Fill in the goal state information
  goal_state_vec.resize(12, 0);
  vectorToFullState(goal_state_vec, goal_state_);

  // Zero planning data
  start_index_ = 0;
  triggerReset();
}

void GlobalBodyPlanner::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);  // Takes ~10ms
  planner_config_.terrain_grid_map = map;            // Takes ~0.1ms

<<<<<<< HEAD
=======
<<<<<<< HEAD
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
  // Uodate the goal state of the planner
  goal_state_.pos[2] =
      planner_config_.h_nom + planner_config_.terrain.getGroundHeight(
                                  goal_state_.pos[0], goal_state_.pos[1]);
}
<<<<<<< HEAD
=======
=======
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
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

void GlobalBodyPlanner::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  eigenToFullState(quad_utils::bodyStateMsgToEigen(msg->body), robot_state_);
}

void GlobalBodyPlanner::triggerReset() {
  planner_status_ = RESET;
  current_plan_.clear();
  reset_time_ = ros::Time::now();
}

void GlobalBodyPlanner::goalStateCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  // If same as previous goal state, ignore
  if (goal_state_msg_ != NULL) {
    if (goal_state_msg_->header.stamp == msg->header.stamp) {
      return;
    }
  }

  // Load the message
  goal_state_msg_ = msg;

  // Store the x and y locations along with the terrain height (this will be
  // overriden)
  goal_state_.pos[0] = goal_state_msg_->point.x;
  goal_state_.pos[1] = goal_state_msg_->point.y;
  goal_state_.pos[2] = planner_config_.h_nom +
                       planner_config_.terrain.getGroundHeight(
                           goal_state_msg_->point.x, goal_state_msg_->point.y);

  // Invalidate the current plan to force a new one
  current_plan_.invalidate();

  // If the old plan has been executed, allow full replanning, otherwise
  // immediately update plan
  if (current_plan_.getDuration() <=
      (ros::Time::now() - current_plan_.getPublishedTimestamp()).toSec()) {
    triggerReset();
  }
}

void GlobalBodyPlanner::setStartState() {
  // Reset if too far from plan
  if (!current_plan_.isEmpty() && !publish_after_reset_delay_) {
    int current_index;
    double first_element_duration;
    quad_utils::getPlanIndex(current_plan_.getPublishedTimestamp(), dt_,
                             current_index, first_element_duration);
    current_index = std::min(current_index, current_plan_.getSize() - 1);
    FullState current_state_in_plan_ =
        current_plan_.getStateFromIndex(current_index);
    if (poseDistance(robot_state_, current_state_in_plan_) >
        pos_error_threshold_) {
      ROS_WARN_THROTTLE(0.5, "Too far from nominal plan, resetting");
      triggerReset();
    }
  }

  if (planner_status_ == RESET) {
    ROS_INFO_THROTTLE(2, "In reset mode");
    start_state_ = robot_state_;
    replan_start_time_ = 0;
    start_index_ = 0;
    publish_after_reset_delay_ = true;

  } else if (planner_status_ == REFINE) {
    ROS_INFO_THROTTLE(2, "GBP in refine mode");

    start_index_ =
        std::floor((ros::Time::now() + ros::Duration(max_planning_time_) -
                    current_plan_.getPublishedTimestamp())
                       .toSec() /
                   dt_);

    // Ensure start index is not too close to goal
    start_index_ = (start_index_ + 25 >= current_plan_.getSize() - 1)
                       ? current_plan_.getSize() - 1
                       : start_index_;

    // Iterate until start_index is in a connect phase
    while (current_plan_.getPrimitiveFromIndex(start_index_) != CONNECT &&
           start_index_ < current_plan_.getSize() - 1) {
      start_index_++;
    }

    start_state_ = current_plan_.getStateFromIndex(start_index_);
    replan_start_time_ = current_plan_.getTime(start_index_);

  } else {
    ROS_ERROR("Invalid planning status");
  }
}

void GlobalBodyPlanner::setGoalState() {}

bool GlobalBodyPlanner::callPlanner() {
  if (!replanning_allowed_ && !publish_after_reset_delay_) {
    newest_plan_.setComputedTimestamp(ros::Time::now());
    return false;
  }

  newest_plan_ = current_plan_;

  // Clear out old statistics
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vector_.clear();
  cost_vector_times_.clear();

  // Copy start and goal states and adjust for ground height
  State start_state = fullStateToState(start_state_);
  State goal_state = fullStateToState(goal_state_);

  // Initialize statistics variables
  double plan_time, path_length, path_duration, total_solve_time,
      total_vertices_generated, total_path_length, total_path_duration,
      dist_to_goal;
  int vertices_generated;

  // Construct RRT object
  GBPL gbpl;

  // Loop through num_calls_ planner calls
  for (int i = 0; i < num_calls_; ++i) {
    // Exit if ros is down
    if (!ros::ok()) {
      return false;
    }

    // Clear out previous solutions and initialize new statistics variables
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    // Call the planner method
    int plan_status = gbpl.findPlan(planner_config_, start_state, goal_state,
                                    state_sequence, action_sequence, tree_pub_);
    newest_plan_.setComputedTimestamp(ros::Time::now());

    if (plan_status != VALID && plan_status != VALID_PARTIAL) {
      if (plan_status == INVALID_START_STATE) {
        ROS_WARN_THROTTLE(1, "Invalid start state, exiting");
      } else if (plan_status == INVALID_GOAL_STATE) {
        ROS_WARN_THROTTLE(1, "Invalid goal state, exiting");
      } else if (plan_status == INVALID_START_GOAL_EQUAL) {
        ROS_WARN_THROTTLE(1, "Start is sufficiently close to goal, exiting");
      } else if (plan_status == UNSOLVED) {
        ROS_WARN_THROTTLE(1,
                          "Planner was unable to make any progress, start "
                          "state likely trapped");
      }
      return false;
    }
    gbpl.getStatistics(plan_time, vertices_generated, path_length,
                       path_duration, dist_to_goal);

    // Add the existing path length to the new
    path_length += current_plan_.getLengthAtIndex(start_index_);

    // Handle the statistical data
    cost_vector_.push_back(path_length);
    cost_vector_times_.push_back(plan_time);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_length += path_length;
    total_path_duration += path_duration;

    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);

    newest_plan_.eraseAfterIndex(start_index_);
    newest_plan_.loadPlanData(plan_status, start_state_, dist_to_goal,
                              state_sequence, action_sequence, dt_,
                              replan_start_time_, planner_config_);

    // Check if this plan is better:
    // 1) If valid and shorter or previous plan not valid OR
    // 2) If partially valid and closer to the goal OR
    // 3) If goal has moved
    double eps = 0.99;  // Require significant improvement
    bool is_updated = false;
    if ((plan_status == VALID) &&
        ((newest_plan_.getLength() / eps) < current_plan_.getLength() ||
         current_plan_.getStatus() != VALID)) {
      ROS_INFO("valid and shorter or previous plan not valid");
      is_updated = true;

    } else if ((plan_status == VALID_PARTIAL) &&
               (current_plan_.getStatus() == UNSOLVED ||
                (poseDistance(state_sequence.back(), goal_state) <
                 current_plan_.getGoalDistance()))) {
      ROS_INFO("partially valid and closer to the goal");
      is_updated = true;
    }

    if (is_updated) {
      state_sequence_ = state_sequence;
      action_sequence_ = action_sequence;

      std::cout << "Solve time: " << plan_time << " s" << std::endl;
      std::cout << "Vertices generated: " << vertices_generated << std::endl;
      std::cout << "Path length: " << path_length << " m" << std::endl;
      std::cout << "Path duration: " << path_duration << " s" << std::endl;
      std::cout << std::endl;

      current_plan_ = newest_plan_;
    }

    return is_updated;
  }

  // Report averaged statistics if num_calls_ > 1
  if (num_calls_ > 1) {
    std::cout << "Average vertices generated: "
              << total_vertices_generated / num_calls_ << std::endl;
    std::cout << "Average solve time: " << total_solve_time / num_calls_ << " s"
              << std::endl;
    std::cout << "Average path length: " << total_path_length / num_calls_
              << " s" << std::endl;
    std::cout << "Average path duration: " << total_path_duration / num_calls_
              << " s" << std::endl;
    std::cout << std::endl;
  }
<<<<<<< HEAD
=======
<<<<<<< HEAD
=======

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

>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
}

void GlobalBodyPlanner::waitForData() {
  // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while ((shared_map == nullptr) && ros::ok()) {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(
        terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<quad_msgs::RobotState const> shared_robot_state;
<<<<<<< HEAD
  while ((shared_robot_state == nullptr) && ros::ok()) {
    shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(
        robot_state_topic_, nh_);
=======
<<<<<<< HEAD
  while ((shared_robot_state == nullptr) && ros::ok()) {
    shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(
        robot_state_topic_, nh_);
=======
  while((shared_robot_state == nullptr) && ros::ok())
  {
    shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(robot_state_topic_, nh_);
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
    ros::spinOnce();
  }
  ROS_INFO("GBP has state and map information");
  reset_time_ = ros::Time::now();
}

void GlobalBodyPlanner::getInitialPlan() {
  // Keep track of when the planner started
  ros::Time start_time = ros::Time::now();

  bool success = false;

  // Repeatedly call the planner until the startup delay has elapsed
  while (ros::ok() && ((ros::Time::now() - start_time) <
                       ros::Duration(reset_publish_delay_))) {
    success = callPlanner();
  }
}

void GlobalBodyPlanner::publishCurrentPlan() {
  // Conditions for publishing current plan:
  // 1) Plan not empty AND
  // 2) Reset publish delay has passed AND
  // 3) One of the following conditions is met:
  //    a) Current plan not yet published after reset
  //    b) The new plan is the best plan

  // Check conditions 1) and 2) return if false
  if (current_plan_.isEmpty() ||
      ((ros::Time::now() - reset_time_).toSec() <= reset_publish_delay_))
    return;

  // Check condition 3
  if (publish_after_reset_delay_ || newest_plan_ == current_plan_) {
    // If this is a reset, update the timestamp and switch back to refinement
    // mode
    if (publish_after_reset_delay_) {
      ROS_INFO("Switching to refinement mode");
      current_plan_.setPublishedTimestamp(ros::Time::now());
      planner_status_ = REFINE;
      publish_after_reset_delay_ = false;
    }
<<<<<<< HEAD

    // Declare the messages for interpolated body plan and discrete states,
    // initialize their headers
    quad_msgs::RobotPlan robot_plan_msg;
    quad_msgs::RobotPlan discrete_robot_plan_msg;
    robot_plan_msg.header.frame_id = map_frame_;
    robot_plan_msg.header.stamp = ros::Time::now();
    discrete_robot_plan_msg.header = robot_plan_msg.header;

    // Initialize the headers and types
    robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();
    discrete_robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();

    // Load the plan into the messages
    current_plan_.convertToMsg(robot_plan_msg, discrete_robot_plan_msg);

    // Publish both messages
    body_plan_pub_.publish(robot_plan_msg);
    discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

    ROS_WARN("New plan published, stamp = %f",
             robot_plan_msg.global_plan_timestamp.toSec());
  }
}

=======

    // Declare the messages for interpolated body plan and discrete states,
    // initialize their headers
    quad_msgs::RobotPlan robot_plan_msg;
    quad_msgs::RobotPlan discrete_robot_plan_msg;
    robot_plan_msg.header.frame_id = map_frame_;
    robot_plan_msg.header.stamp = ros::Time::now();
    discrete_robot_plan_msg.header = robot_plan_msg.header;

    // Initialize the headers and types
    robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();
    discrete_robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();

    // Load the plan into the messages
    current_plan_.convertToMsg(robot_plan_msg, discrete_robot_plan_msg);

    // Publish both messages
    body_plan_pub_.publish(robot_plan_msg);
    discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

    ROS_WARN("New plan published, stamp = %f",
             robot_plan_msg.global_plan_timestamp.toSec());
  }
}

>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
void GlobalBodyPlanner::spin() {
  ros::Rate r(update_rate_);

  // Wait until we get map and state data
  waitForData();

  // Enter main spin
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();

    // Set the start and goal states
    setStartState();
    setGoalState();

    // Call the planner
    callPlanner();

    // Publish the results if valid
    publishCurrentPlan();

    r.sleep();
  }
}
