/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <chrono>

#include "global_body_planner/planner_class.h"
#include "global_body_planner/rrt_star_connect.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <spirit_msgs/BodyPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_ros/GridMapRosConverter.hpp"

/* Input Arguments */
#define ROBOTSTART_IN prhs[0]
#define ROBOTGOAL_IN     prhs[1]
#define XDATA_IN      prhs[2]
#define YDATA_IN      prhs[3]
#define ZDATA_IN    prhs[4]
#define DXDATA_IN     prhs[5]
#define DYDATA_IN     prhs[6]
#define DZDATA_IN   prhs[7]

/* Output Arguments */
#define STATES_OUT  plhs[0]
#define ACTIONS_OUT plhs[1]
#define TIMES_OUT plhs[2]
#define VERTICES_OUT  plhs[3]
#define COSTS_OUT   plhs[4]
#define COST_TIMES_OUT    plhs[5]

Ground ground_copy;

static void planner(
       Ground &ground,
       State robot_start,
       State robot_goal,
       std::vector<State> &state_sequence,
       std::vector<Action> &action_sequence,
       std::vector<double> &solve_time_info,
       std::vector<int> &vertices_generated_info,
       std::vector<std::vector<double> > &cost_vectors,
       std::vector<std::vector<double> > &cost_vectors_times)
{
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

  cost_vectors.reserve(N);
  cost_vectors_times.reserve(N);

  // auto t_start = std::chrono::high_resolution_clock::now();

  RRTClass rrt_obj;
  RRTConnectClass rrt_connect_obj;
  RRTStarConnectClass rrt_star_connect_obj;


  for (int i = 0; i<N; ++i)
  {
    state_sequence.clear();
    action_sequence.clear();

    std::vector<double> cost_vector;
    std::vector<double> cost_vector_times;

    // rrt_obj.buildRRT(ground, robot_start, robot_goal,state_sequence,action_sequence);
    // rrt_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);

    rrt_connect_obj.buildRRTConnect(ground, robot_start, robot_goal,state_sequence,action_sequence, max_time);
    rrt_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);
    
    // rrt_star_connect_obj.buildRRTStarConnect(ground, robot_start, robot_goal,state_sequence,action_sequence, max_time);
    // rrt_star_connect_obj.getStatistics(plan_time,success, vertices_generated, time_to_first_solve, cost_vector, cost_vector_times, path_duration);

    // std::cout << "made it out" << std::endl;

    cost_vectors.push_back(cost_vector);
    cost_vectors_times.push_back(cost_vector_times);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_duration += path_duration;

    std::cout << "Vertices generated: " << vertices_generated << std::endl;
    std::cout << "Solve time: " << plan_time << std::endl;
    std::cout << "Time to first solve: " << time_to_first_solve << std::endl;
    std::cout << "Path length: " << cost_vector.back() << std::endl;

    // solve_time_info.push_back(time_to_first_solve);
    solve_time_info.push_back(plan_time);
    vertices_generated_info.push_back(vertices_generated);
  }
    
  if (N > 1)
  {
    std::cout << "Average vertices generated: " << total_vertices_generated/N << std::endl;
    std::cout << "Average solve time: " << total_solve_time/N << std::endl;
    std::cout << "Average path duration: " << total_path_duration/N << std::endl;
  }
}

void chatterCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);

  int x_size = map.getSize()(0);
  int y_size = map.getSize()(1);
  std::vector<double> x_data(x_size);
  std::vector<double> y_data(y_size);
  std::vector<std::vector<double> > z_data(x_size);
  std::vector<std::vector<double> > dx_data(x_size);
  std::vector<std::vector<double> > dy_data(x_size);
  std::vector<std::vector<double> > dz_data(x_size);

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

  // for(double x : x_data)
  //  std::cout << "x = " << x << std::endl;
  // for(double y : y_data)
  //  std::cout << "y = " << y << std::endl;

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

  ground_copy.x_size = x_size;
  ground_copy.y_size = y_size;
  ground_copy.x_data = x_data;
  ground_copy.y_data = y_data;
  ground_copy.z_data = z_data;
  ground_copy.dx_data = dx_data;
  ground_copy.dy_data = dy_data;
  ground_copy.dz_data = dz_data;
  

  for (int row=0; row < x_size; ++row)
  {
    for (int col=0; col < y_size; ++col)
    {
      z_data[row].push_back(0.0);
      dx_data[row].push_back(0.0);
      dy_data[row].push_back(0.0);
      dz_data[row].push_back(1.0);
    }
  }

  

  // std::vector<double> x_data;
  // std::vector<double> y_data;
  // std::vector<double> z_data;

  // grid_map::Matrix& data = map["elevation"];
  // for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  // {
  //     const grid_map::Index index(*iterator);
  //     grid_map::Position position;
 //       map.getPosition(*iterator, position);
  //     std::cout << "The position at index " << index.transpose() << " is " << position(0) << "," << position(1) << std::endl;
  //     std::cout << "The value at index " << index.transpose() << " is " << data(index(0), index(1)) << std::endl;
  // }
}

int main(int argc, char *argv[])
{
  printf( "\nHello World! Made it in.\n\n" );

  // initialize ROS and the node
  ros::init(argc, argv, "global_body_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
asdfasdfasdf
  // configuring parameters
  std::string map_frame;
  double publish_frequency = 10;
  ros::Publisher plan_pub;
  ros::Publisher plan_pub_viz;
  ros::Publisher discrete_states_pub;

  nh.param<std::string>("map_frame",map_frame,"/map");

  plan_pub = nh.advertise<spirit_msgs::BodyPlan>("/body_plan", 1);
  // plan_pub_viz = nh.advertise<nav_msgs::Path>("visualization/body_plan", 1);
  discrete_states_pub = nh.advertise<visualization_msgs::Marker>("discrete_states", 1);
  ros::Subscriber map_sub = nh.subscribe("/terrain_map", 1, chatterCallback);

  ros::Rate rate(publish_frequency);
  
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while(shared_map == nullptr)
  {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>("/terrain_map", nh);
    ros::spinOnce();
    rate.sleep();
  }

  Ground ground;
  ground = ground_copy;

  State robot_start = {0,0,0.3,0.5,0,0,0,0};
  State robot_goal =  {8,0.0,0.3,0,0.5,0,0,0};
  
  std::vector<State> state_sequence;
  std::vector<Action> action_sequence;
  std::vector<std::vector<double> > cost_vectors;
  std::vector<std::vector<double> > cost_vectors_times;
  std::vector<double> solve_time_info;
  std::vector<int> vertices_generated_info;

  //call the planner
  planner(ground, robot_start, robot_goal, state_sequence, action_sequence, solve_time_info, vertices_generated_info, cost_vectors, cost_vectors_times); 

  int state_length = state_sequence.size();
  int action_length = action_sequence.size();

  double dt = 0.05;
  std::vector<State> interp_path;
  std::vector<double> interp_t;
  std::vector<int> interp_phase;
  getInterpPath(state_sequence, action_sequence,dt,interp_path, interp_t, interp_phase);

  // printStateSequence(state_sequence);
  // printInterpStateSequence(interp_path, interp_t);
  // printActionSequence(action_sequence);


  while (nh.ok())
  {
    // construct a pose message
    spirit_msgs::BodyPlan body_plan;
    visualization_msgs::Marker discrete_states;

    ros::Time timestamp = ros::Time::now();
    body_plan.header.stamp = timestamp;
    body_plan.header.frame_id = map_frame;
    discrete_states.header.stamp = timestamp;
    discrete_states.header.frame_id = map_frame;
    discrete_states.id = 0;
    discrete_states.type = visualization_msgs::Marker::POINTS;
    
    double scale = 0.2;
    discrete_states.scale.x = scale;
    discrete_states.scale.y = scale;
    discrete_states.scale.z = scale;
    discrete_states.color.r = 0.733f;
    discrete_states.color.a = 1.0;

    for (int i=0;i<interp_path.size(); ++i)
    {
      ros::Duration time_elapsed(interp_t[i]);
      ros::Time current_time = timestamp + time_elapsed;

      nav_msgs::Odometry state;
      state.header.frame_id = map_frame;
      state.header.stamp = current_time;
      state.child_frame_id = "dummy";

      tf2::Quaternion quat_tf;
      geometry_msgs::Quaternion quat_msg;
      quat_tf.setRPY(0,interp_path[i][6], atan2(interp_path[i][4],interp_path[i][3]));
      quat_msg = tf2::toMsg(quat_tf);

      state.pose.pose.position.x = interp_path[i][0];
      state.pose.pose.position.y = interp_path[i][1];
      state.pose.pose.position.z = interp_path[i][2];
      state.pose.pose.orientation = quat_msg;

      state.twist.twist.linear.x = interp_path[i][3];
      state.twist.twist.linear.y = interp_path[i][4];
      state.twist.twist.linear.z = interp_path[i][5];
      state.twist.twist.angular.x = 0;
      state.twist.twist.angular.y = interp_path[i][6];
      state.twist.twist.angular.z = 0;

      body_plan.states.push_back(state);

      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = current_time;

      pose_stamped.pose.position.x = state.pose.pose.position.x;
      pose_stamped.pose.position.y = state.pose.pose.position.y;
      pose_stamped.pose.position.z = state.pose.pose.position.z;
      pose_stamped.pose.orientation = state.pose.pose.orientation;

    }

    for (int i = 0; i<state_sequence.size(); i++)
    {
      geometry_msgs::Point p;
      p.x = state_sequence[i][0];
      p.y = state_sequence[i][1];
      p.z = state_sequence[i][2];
      discrete_states.points.push_back(p);
    }

    plan_pub.publish(body_plan);
    discrete_states_pub.publish(discrete_states);

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;


}

