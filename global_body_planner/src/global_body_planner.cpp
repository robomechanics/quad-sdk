#include "global_body_planner/global_body_planner.hpp"

#include <chrono>

using namespace planning_utils;

GlobalBodyPlanner::GlobalBodyPlanner(rclcpp::Node::SharedPtr node)
    : node_(node) {
  // Load rosparams from parameter server
  std::string body_plan_topic, discrete_body_plan_topic, body_plan_tree_topic,
      goal_state_topic;
  std::vector<double> goal_state_vec(2);

  quad_utils::loadROSParam(node_, "topics.start_state", robot_state_topic_);
  quad_utils::loadROSParam(node_, "topics.goal_state", goal_state_topic);
  quad_utils::loadROSParam(node_, "topics.terrain_map", terrain_map_topic_);
  quad_utils::loadROSParam(node_, "topics.global_plan", body_plan_topic);
  quad_utils::loadROSParam(node_, "topics.global_plan_discrete",
                           discrete_body_plan_topic);
  quad_utils::loadROSParam(node_, "topics.global_plan_tree",
                           body_plan_tree_topic);
  quad_utils::loadROSParam(node_, "map_frame", map_frame_);
  quad_utils::loadROSParam(node_, "global_body_planner.update_rate",
                           update_rate_);
  quad_utils::loadROSParam(node_, "global_body_planner.num_calls", num_calls_);
  quad_utils::loadROSParam(node_, "global_body_planner.max_planning_time",
                           max_planning_time_);
  quad_utils::loadROSParam(node_, "global_body_planner.pos_error_threshold",
                           pos_error_threshold_);
  quad_utils::loadROSParam(node_, "global_body_planner.startup_delay",
                           reset_publish_delay_);
  quad_utils::loadROSParam(node_, "global_body_planner.replanning",
                           replanning_allowed_);
  quad_utils::loadROSParam(node_, "local_planner.timestep", dt_);
  quad_utils::loadROSParam(node_, "global_body_planner.goal_state",
                           goal_state_vec);

  // Setup pubs and subs
  terrain_map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
      terrain_map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
      std::bind(&GlobalBodyPlanner::terrainMapCallback, this,
                std::placeholders::_1));
  robot_state_sub_ = node_->create_subscription<quad_msgs::msg::RobotState>(
      robot_state_topic_, 10,
      std::bind(&GlobalBodyPlanner::robotStateCallback, this,
                std::placeholders::_1));
  goal_state_sub_ =
      node_->create_subscription<geometry_msgs::msg::PointStamped>(
          goal_state_topic, 10,
          std::bind(&GlobalBodyPlanner::goalStateCallback, this,
                    std::placeholders::_1));
  body_plan_pub_ =
      node_->create_publisher<quad_msgs::msg::RobotPlan>(body_plan_topic, 10);
  discrete_body_plan_pub_ = node_->create_publisher<quad_msgs::msg::RobotPlan>(
      discrete_body_plan_topic, 10);
  tree_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      body_plan_tree_topic, 10);
  goal_reached_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);

  // Note: the plan_with_constraints service is intentionally NOT advertised
  // here. waitForData() inside spin() calls rclcpp::spin_some() repeatedly
  // while it waits for the first terrain map and state messages, and that
  // spin would dispatch any incoming service requests against
  // uninitialized robot_state_. See spin() for the deferred advertisement.

  // Load planner config
  bool enable_leaping;
  planner_config_.loadParamsFromServer(node_);
  enable_leaping = node_->declare_parameter<bool>(
      "global_body_planner.enable_leaping", true);
  if (!enable_leaping) {
    planner_config_.enable_leaping = false;
    planner_config_.num_leap_samples = 0;
    planner_config_.h_min = 0;
    planner_config_.h_max = 0.5;
  }

  // In CBS mode, defer planning to plan_with_constraints service calls.
  // Default false preserves single-robot spin-loop planning.
  cbs_mode_ = node_->declare_parameter<bool>(
      "global_body_planner.cbs_mode", false);

  // Fill in the goal state information
  goal_state_vec.resize(12, 0);
  vectorToFullState(goal_state_vec, goal_state_);

  // Zero planning data
  start_index_ = 0;
  triggerReset();
}

void GlobalBodyPlanner::terrainMapCallback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);  // Takes ~10ms
  planner_config_.terrain_grid_map = map;            // Takes ~0.1ms

  // Uodate the goal state of the planner
  goal_state_.pos[2] =
      planner_config_.h_nom + planner_config_.terrain.getGroundHeight(
                                  goal_state_.pos[0], goal_state_.pos[1]);
  map_recieved_ = true;
}

void GlobalBodyPlanner::robotStateCallback(
    const quad_msgs::msg::RobotState::SharedPtr msg) {
  eigenToFullState(quad_utils::bodyStateMsgToEigen(msg->body), robot_state_);
}

void GlobalBodyPlanner::triggerReset() {
  planner_status_ = RESET;
  current_plan_.clear();
  reset_time_ = node_->now();
  // The cached RRT trees are tied to a specific start/goal pair; any reset
  // invalidates that assumption, so drop them.
  gbpl_.invalidateCache();
  gbpl_.setWarmStart(false);
}

void GlobalBodyPlanner::goalStateCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {
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
      (node_->now() - current_plan_.getPublishedTimestamp()).seconds()) {
    triggerReset();
  }
}

void GlobalBodyPlanner::setStartState() {
  // Reset if too far from plan
  if (!current_plan_.isEmpty() && !publish_after_reset_delay_) {
    int current_index;
    double first_element_duration;
    quad_utils::getPlanIndex(node_, current_plan_.getPublishedTimestamp(), dt_,
                             current_index, first_element_duration);
    current_index = std::min(current_index, current_plan_.getSize() - 1);
    FullState current_state_in_plan_ =
        current_plan_.getStateFromIndex(current_index);
    if (poseDistance(robot_state_, current_state_in_plan_) >
        pos_error_threshold_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Too far from nominal plan, resetting");
      triggerReset();
    }
  }

  if (planner_status_ == RESET) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "In reset mode");
    start_state_ = robot_state_;
    replan_start_time_ = 0;
    start_index_ = 0;
    publish_after_reset_delay_ = true;

  } else if (planner_status_ == REFINE) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "GBP in refine mode");

    start_index_ = std::floor(
        (node_->now() + rclcpp::Duration::from_seconds(max_planning_time_) -
         current_plan_.getPublishedTimestamp())
            .seconds() /
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
    RCLCPP_ERROR(node_->get_logger(), "Invalid planning status");
  }
}

void GlobalBodyPlanner::setGoalState() {}

bool GlobalBodyPlanner::callPlanner() {
  if (!replanning_allowed_ && !publish_after_reset_delay_) {
    newest_plan_.setComputedTimestamp(node_->now());
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

  // RRT-Connect planner kept as a class member so that warm-started CBS
  // service calls reuse the trees from the previous solve.

  // Loop through num_calls_ planner calls
  for (int i = 0; i < num_calls_; ++i) {
    // Exit if ros is down
    if (!rclcpp::ok()) {
      return false;
    }

    // Clear out previous solutions and initialize new statistics variables
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    // Call the planner method
    int plan_status = gbpl_.findPlan(planner_config_, start_state, goal_state,
                                     state_sequence, action_sequence,
                                     tree_pub_);
    newest_plan_.setComputedTimestamp(node_->now());

    if (plan_status != VALID && plan_status != VALID_PARTIAL) {
      if (plan_status == INVALID_START_STATE) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Invalid start state, exiting");
      } else if (plan_status == INVALID_GOAL_STATE) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Invalid goal state, exiting");
      } else if (plan_status == INVALID_START_GOAL_EQUAL) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Start is sufficiently close to goal, exiting");
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        goal_reached_pub_->publish(msg);
      } else if (plan_status == UNSOLVED) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Planner was unable to make any progress, start "
                             "state likely trapped");
      }
      return false;
    }
    gbpl_.getStatistics(plan_time, vertices_generated, path_length,
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
      RCLCPP_INFO(node_->get_logger(),
                  "valid and shorter or previous plan not valid");
      is_updated = true;

    } else if ((plan_status == VALID_PARTIAL) &&
               (current_plan_.getStatus() == UNSOLVED ||
                (poseDistance(state_sequence.back(), goal_state) <
                 current_plan_.getGoalDistance()))) {
      RCLCPP_INFO(node_->get_logger(),
                  "partially valid and closer to the goal");
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
}

void GlobalBodyPlanner::waitForData() {
  // Spin until terrain map message has been received and processed
  grid_map_msgs::msg::GridMap map_msg;
  while (!map_recieved_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }

  quad_msgs::msg::RobotState state_msg;
  bool got_state = false;

  while (!got_state && rclcpp::ok()) {
    got_state = rclcpp::wait_for_message(state_msg, node_, robot_state_topic_,
                                         std::chrono::seconds(3));
    rclcpp::spin_some(node_);
  }
  RCLCPP_INFO(node_->get_logger(), "GBP has state and map information");
  reset_time_ = node_->now();
}

void GlobalBodyPlanner::getInitialPlan() {
  // Keep track of when the planner started
  rclcpp::Time start_time = node_->now();

  bool success = false;

  // Repeatedly call the planner until the startup delay has elapsed
  while (rclcpp::ok() &&
         ((node_->now() - start_time) <
          rclcpp::Duration::from_seconds(reset_publish_delay_))) {
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
      ((node_->now() - reset_time_).seconds() <= reset_publish_delay_))
    return;

  // Check condition 3
  if (publish_after_reset_delay_ || newest_plan_ == current_plan_) {
    // If this is a reset, update the timestamp and switch back to refinement
    // mode
    if (publish_after_reset_delay_) {
      RCLCPP_INFO(node_->get_logger(), "Switching to refinement mode");
      current_plan_.setPublishedTimestamp(node_->now());
      planner_status_ = REFINE;
      publish_after_reset_delay_ = false;
    }

    // Declare the messages for interpolated body plan and discrete states,
    // initialize their headers
    quad_msgs::msg::RobotPlan robot_plan_msg;
    quad_msgs::msg::RobotPlan discrete_robot_plan_msg;
    robot_plan_msg.header.frame_id = map_frame_;
    robot_plan_msg.header.stamp = node_->now();
    discrete_robot_plan_msg.header = robot_plan_msg.header;

    // Initialize the headers and types
    robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();
    discrete_robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();

    // Load the plan into the messages
    current_plan_.convertToMsg(robot_plan_msg, discrete_robot_plan_msg);

    // Publish both messages
    body_plan_pub_->publish(robot_plan_msg);
    discrete_body_plan_pub_->publish(discrete_robot_plan_msg);

    RCLCPP_WARN(node_->get_logger(), "New plan published, stamp = %f",
                rclcpp::Time(robot_plan_msg.global_plan_timestamp).seconds());
  }
}

void GlobalBodyPlanner::spin() {
  rclcpp::Rate r(update_rate_);

  // Wait until we get map and state data
  waitForData();

  // Advertise after waitForData() so service callbacks see initialized state.
  // The conflict_based_search node calls this service for constrained replans.
  plan_with_constraints_srv_ =
      node_->create_service<quad_msgs::srv::PlanWithConstraints>(
          "plan_with_constraints",
          std::bind(&GlobalBodyPlanner::planWithConstraintsCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Enter main spin
  while (rclcpp::ok()) {
    // Process callbacks
    rclcpp::spin_some(node_);

    // In CBS mode, service callbacks own constrained planning and publishing.
    if (cbs_mode_) {
      r.sleep();
      continue;
    }

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

void GlobalBodyPlanner::planWithConstraintsCallback(
    const std::shared_ptr<quad_msgs::srv::PlanWithConstraints::Request> request,
    std::shared_ptr<quad_msgs::srv::PlanWithConstraints::Response> response) {

  cbs_mode_ = true;

  // Attach service-scope dynamic collision constraints to the planner config.
  const auto& c = request->constraints;
  planner_config_.dynamic_constraints.clear();
  const size_t n = std::min({c.pos_x.size(), c.pos_y.size(), c.pos_z.size(),
                             c.yaw.size(), c.t_start.size(), c.t_end.size()});
  planner_config_.dynamic_constraints.reserve(n);
  // Use request extents when provided; otherwise default to this robot's body.
  const double l = (c.length > 0.0) ? c.length : planner_config_.robot_l;
  const double w = (c.width > 0.0) ? c.width : planner_config_.robot_w;
  const double h = (c.height > 0.0) ? c.height : planner_config_.robot_h;
  for (size_t i = 0; i < n; ++i) {
    planning_utils::TimedPoseConstraint constraint;
    constraint.pos = Eigen::Vector3d(c.pos_x[i], c.pos_y[i], c.pos_z[i]);
    constraint.yaw = c.yaw[i];
    constraint.t_start = c.t_start[i];
    constraint.t_end = c.t_end[i];
    constraint.half_extents = Eigen::Vector3d(0.5 * l, 0.5 * w, 0.5 * h);
    planner_config_.dynamic_constraints.push_back(constraint);
  }

  gbpl_.setWarmStart(request->warm_start);

  triggerReset();

  // triggerReset clears warm-start state.
  gbpl_.setWarmStart(request->warm_start);
  setStartState();
  setGoalState();

  planning_utils::resetValidityStats();
  const auto t_solve_start = std::chrono::steady_clock::now();
  bool success = callPlanner();
  const auto t_solve_end = std::chrono::steady_clock::now();
  const auto solve_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          t_solve_end - t_solve_start)
          .count();
  const auto stats = planning_utils::getValidityStats();
  const double reject_pct =
      stats.total > 0
          ? 100.0 * stats.constraint_rejects / stats.total
          : 0.0;
  RCLCPP_INFO(
      node_->get_logger(),
      "plan_with_constraints: constraints=%zu | validity=%d | "
      "constraint_rejects=%d (%.1f%%) | solve=%lldms | success=%d | "
      "plan_states=%d",
      planner_config_.dynamic_constraints.size(), stats.total,
      stats.constraint_rejects, reject_pct, static_cast<long long>(solve_ms),
      success, current_plan_.getSize());
  // Do not publish intermediate CBS solves; CBS publishes the final plan set.

  // Build the response from current_plan_ (which callPlanner has just
  // populated).
  quad_msgs::msg::RobotPlan robot_plan_msg;
  quad_msgs::msg::RobotPlan discrete_plan_msg;
  robot_plan_msg.header.frame_id = map_frame_;
  robot_plan_msg.header.stamp = node_->now();
  discrete_plan_msg.header = robot_plan_msg.header;
  robot_plan_msg.global_plan_timestamp = current_plan_.getPublishedTimestamp();
  discrete_plan_msg.global_plan_timestamp =
      current_plan_.getPublishedTimestamp();
  current_plan_.convertToMsg(robot_plan_msg, discrete_plan_msg);

  response->plan = robot_plan_msg;
  response->path_length = current_plan_.getLength();
  response->success = success;

  // Drop the constraints again so subsequent regular planning calls (the
  // ones running on the spin timer) see a clean config.
  planner_config_.dynamic_constraints.clear();
}
