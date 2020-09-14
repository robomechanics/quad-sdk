#include "local_footstep_planner/local_footstep_planner.h"

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, footstep_plan_topic;
  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<double>("local_footstep_planner/update_rate", update_rate_, 1);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1,&LocalFootstepPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&LocalFootstepPlanner::bodyPlanCallback, this);
  footstep_plan_pub_ = nh_.advertise<visualization_msgs::Marker>(footstep_plan_topic,1);
}

void LocalFootstepPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

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

  terrain_.x_size = x_size;
  terrain_.y_size = y_size;
  terrain_.x_data = x_data;
  terrain_.y_data = y_data;
  terrain_.z_data = z_data;
  terrain_.dx_data = dx_data;
  terrain_.dy_data = dy_data;
  terrain_.dz_data = dz_data;
}

void LocalFootstepPlanner::bodyPlanCallback(const nav_msgs::Path::ConstPtr& msg) {
  int length = msg->poses.size();
  for (int i=0; i < length; i++) {
    BodyState s;
    s[0] = msg->poses[i].pose.position.x;
    s[1] = msg->poses[i].pose.position.y;
    s[2] = msg->poses[i].pose.position.z;
    body_plan_.push_back(s);
  }
}

void LocalFootstepPlanner::updatePlan() {
  // for (int i=0; i<body_plan_.size(); i++)
  // {
    
  // }
}

void LocalFootstepPlanner::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    ROS_INFO("In spin, updating at %4.1f Hz", update_rate_);
    ros::spinOnce();
    r.sleep();
  }
}