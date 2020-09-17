#include "global_body_planner/global_body_planner.h"
#include "global_body_planner/planner_class.h"
#include "global_body_planner/rrt_star_connect.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic;
  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("global_body_planner/update_rate", update_rate_, 1);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1,&GlobalBodyPlanner::terrainMapCallback, this);
  body_plan_pub_ = nh_.advertise<spirit_msgs::BodyPlan>(body_plan_topic,1);
  discrete_states_pub_ = nh.advertise<visualization_msgs::Marker>("discrete_states", 1);
}

void GlobalBodyPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Initialize the data structures for the map
  int x_size = map.getSize()(0);
  int y_size = map.getSize()(1);
  std::vector<double> x_data(x_size);
  std::vector<double> y_data(y_size);
  std::vector<std::vector<double> > z_data(x_size);
  std::vector<std::vector<double> > dx_data(x_size);
  std::vector<std::vector<double> > dy_data(x_size);
  std::vector<std::vector<double> > dz_data(x_size);

  // Load the x and y data coordinates
  for (int i=0; i<x_size; i++)
  {
    grid_map::Index index = {(x_size-1)-i, 0};
    grid_map::Position position;
    map.getPosition(index, position);
    x_data[i] = position.x();
  }
  for (int i=0; i<y_size; i++)
  {
    grid_map::Index index = {0, (y_size-1)-i};
    grid_map::Position position;
    map.getPosition(index, position);
    y_data[i] = position.y();;
  }

  // Loop through the map and get the height and slope info
  for (int i=0; i<x_size; i++)
  {
    for (int j=0; j<y_size; j++)
    {
      grid_map::Index index = {(x_size-1)-i, (y_size-1)-j};
      double height = (double) map.at("elevation", index);
      z_data[i].push_back(height);

      dx_data[i].push_back(0.0);
      dy_data[i].push_back(0.0);
      dz_data[i].push_back(1.0);
    }
  }

  // Update the private terrain member
  terrain_.x_size = x_size;
  terrain_.y_size = y_size;
  terrain_.x_data = x_data;
  terrain_.y_data = y_data;
  terrain_.z_data = z_data;
  terrain_.dx_data = dx_data;
  terrain_.dy_data = dy_data;
  terrain_.dz_data = dz_data;
}

void GlobalBodyPlanner::planner() {

  double plan_time;
  int success;
  int vertices_generated;
  double time_to_first_solve;
  double path_duration;

  double total_solve_time = 0;
  double total_vertices_generated = 0;
  double total_path_duration = 0;
  double max_time = 0.0;
  int N = 1;

  cost_vectors_.reserve(N);
  cost_vectors_times_.reserve(N);

  // auto t_start = std::chrono::high_resolution_clock::now();

  RRTClass rrt_obj;
  RRTConnectClass rrt_connect_obj;
  RRTStarConnectClass rrt_star_connect_obj;


  for (int i = 0; i<N; ++i)
  {
    state_sequence_.clear();
    action_sequence_.clear();

    std::vector<double> cost_vector;
    std::vector<double> cost_vector_times;

    // rrt_obj.buildRRT(ground, robot_start, robot_goal,state_sequence,action_sequence);
    // rrt_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);

    rrt_connect_obj.buildRRTConnect(terrain_, robot_start_, robot_goal_,state_sequence_,action_sequence_, max_time);
    rrt_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);
    
    // rrt_star_connect_obj.buildRRTStarConnect(ground, robot_start, robot_goal,state_sequence,action_sequence, max_time);
    // rrt_star_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);

    // std::cout << "made it out" << std::endl;

    cost_vectors_.push_back(cost_vector);
    cost_vectors_times_.push_back(cost_vector_times);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_duration += path_duration;

    std::cout << "Vertices generated: " << vertices_generated << std::endl;
    std::cout << "Solve time: " << plan_time << std::endl;
    std::cout << "Time to first solve: " << time_to_first_solve << std::endl;
    std::cout << "Path length: " << cost_vector.back() << std::endl;

    // solve_time_info.push_back(time_to_first_solve);
    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);
  }
    
  if (N > 1)
  {
    std::cout << "Average vertices generated: " << total_vertices_generated/N << std::endl;
    std::cout << "Average solve time: " << total_solve_time/N << std::endl;
    std::cout << "Average path duration: " << total_path_duration/N << std::endl;
  }
}

void GlobalBodyPlanner::updatePlanParams() {
  robot_start_ = {0,0,0.3,0.5,0,0,0,0};
  robot_goal_ =  {8,0.0,0.3,0,0.5,0,0,0};
}

void GlobalBodyPlanner::updatePlan() {
  // Clear old solutions
  body_plan_.clear();
  t_plan_.clear();
  state_sequence_.clear();
  action_sequence_.clear();
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vectors_.clear();
  cost_vectors_times_.clear();

  //call the planner
  planner(); 

  // Interpolate to get full body plan
  double dt = 0.05;
  std::vector<int> interp_phase;
  getInterpPath(state_sequence_, action_sequence_,dt,body_plan_, t_plan_, interp_phase);
}

void GlobalBodyPlanner::publishPlan() {
  // construct a pose message
  spirit_msgs::BodyPlan body_plan_msg;
  visualization_msgs::Marker discrete_states;

  ros::Time timestamp = ros::Time::now();
  body_plan_msg.header.stamp = timestamp;
  body_plan_msg.header.frame_id = map_frame_;
  discrete_states.header.stamp = timestamp;
  discrete_states.header.frame_id = map_frame_;
  discrete_states.id = 0;
  discrete_states.type = visualization_msgs::Marker::POINTS;
  
  double scale = 0.2;
  discrete_states.scale.x = scale;
  discrete_states.scale.y = scale;
  discrete_states.scale.z = scale;
  discrete_states.color.r = 0.733f;
  discrete_states.color.a = 1.0;

  for (int i=0;i<body_plan_.size(); ++i)
  {
    ros::Duration time_elapsed(t_plan_[i]);
    ros::Time current_time = timestamp + time_elapsed;

    nav_msgs::Odometry state;
    state.header.frame_id = map_frame_;
    state.header.stamp = current_time;
    state.child_frame_id = "dummy";

    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    quat_tf.setRPY(0,body_plan_[i][6], atan2(body_plan_[i][4],body_plan_[i][3]));
    quat_msg = tf2::toMsg(quat_tf);

    state.pose.pose.position.x = body_plan_[i][0];
    state.pose.pose.position.y = body_plan_[i][1];
    state.pose.pose.position.z = body_plan_[i][2];
    state.pose.pose.orientation = quat_msg;

    state.twist.twist.linear.x = body_plan_[i][3];
    state.twist.twist.linear.y = body_plan_[i][4];
    state.twist.twist.linear.z = body_plan_[i][5];
    state.twist.twist.angular.x = 0;
    state.twist.twist.angular.y = body_plan_[i][6];
    state.twist.twist.angular.z = 0;

    body_plan_msg.states.push_back(state);
  }

  for (int i = 0; i<state_sequence_.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = state_sequence_[i][0];
    p.y = state_sequence_[i][1];
    p.z = state_sequence_[i][2];
    discrete_states.points.push_back(p);
  }

  body_plan_pub_.publish(body_plan_msg);
  discrete_states_pub_.publish(discrete_states);
}

void GlobalBodyPlanner::spin() {
  ros::Rate r(update_rate_);

  // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while(shared_map == nullptr)
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>("/terrain_map", nh_);
    ros::spinOnce();
    r.sleep();
  }

  updatePlanParams();
  updatePlan();

  while (ros::ok()) {
    ROS_INFO("In GlobalBodyPlanner spin, updating at %4.1f Hz", update_rate_);
    updatePlan();
    publishPlan();
    ros::spinOnce();
    r.sleep();
  }
}