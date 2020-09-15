#include "local_footstep_planner/local_footstep_planner.h"
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, footstep_plan_topic, footstep_plan_viz_topic;
  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<std::string>("topics/visualization/footstep_plan_viz", footstep_plan_viz_topic, "/visualization/footstep_plan_viz");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("local_footstep_planner/update_rate", update_rate_, 1);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1,&LocalFootstepPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&LocalFootstepPlanner::bodyPlanCallback, this);
  footstep_plan_pub_ = nh_.advertise<spirit_msgs::FootstepPlan>(footstep_plan_topic,1);
  footstep_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(footstep_plan_viz_topic,1);
}

void LocalFootstepPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
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

  // Update the private ground member
  terrain_.x_size = x_size;
  terrain_.y_size = y_size;
  terrain_.x_data = x_data;
  terrain_.y_data = y_data;
  terrain_.z_data = z_data;
  terrain_.dx_data = dx_data;
  terrain_.dy_data = dy_data;
  terrain_.dz_data = dz_data;
}

void LocalFootstepPlanner::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  // Loop through the message to get the state info and add to private vector
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    // Convert orientation from quaternion to rpy
    tf::Quaternion q(
        msg->states[i].pose.pose.orientation.x,
        msg->states[i].pose.pose.orientation.y,
        msg->states[i].pose.pose.orientation.z,
        msg->states[i].pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get the time associated with this data
    ros::Duration t_plan = msg->states[i].header.stamp - msg->states[0].header.stamp;
    t_plan_.push_back(t_plan.toSec());

    // Get the state associated with this data
    BodyState s;
    s[0] = msg->states[i].pose.pose.position.x;
    s[1] = msg->states[i].pose.pose.position.y;
    s[2] = msg->states[i].pose.pose.position.z;
    s[3] = msg->states[i].twist.twist.linear.x;
    s[4] = msg->states[i].twist.twist.linear.y;
    s[5] = msg->states[i].twist.twist.linear.z;
    s[6] = pitch;
    s[7] = msg->states[i].twist.twist.angular.z;
    s[8] = yaw;
    body_plan_.push_back(s);
  }
}

void LocalFootstepPlanner::updatePlan() {
  double num_feet = 4;
  double x_offsets[4] = {0.3, -0.3, 0.3, -0.3};
  double y_offsets[4] = {0.2, 0.2, -0.2, -0.2};

  for (int i=0; i <t_plan_.size(); i++) {

    double sy = sin(body_plan_[i][8]);
    double cy = cos(body_plan_[i][8]);

    for (int j=0; j<num_feet; j++) {
      FootstepState footstep;

      footstep[0] = j;
      footstep[1] = body_plan_[i][0] + x_offsets[j]*cy - y_offsets[j]*sy;
      footstep[2] = body_plan_[i][1] + x_offsets[j]*sy + y_offsets[j]*cy;
      footstep[3] = t_plan_[i];
      footstep[4] = 0.2;

      footstep_plan_.push_back(footstep);
    }
  }
}

void LocalFootstepPlanner::publishPlan() {
  spirit_msgs::FootstepPlan footstep_plan_msg;
  ros::Time timestamp = ros::Time::now();
  footstep_plan_msg.header.stamp = timestamp;
  footstep_plan_msg.header.frame_id = map_frame_;

  for (int i=0;i<footstep_plan_.size(); ++i) {
    spirit_msgs::Footstep footstep;

    footstep.index = footstep_plan_[i][0];
    footstep.position.x = footstep_plan_[i][1];
    footstep.position.y = footstep_plan_[i][2];
    footstep.td = ros::Duration(footstep_plan_[i][3]);
    footstep.ts = ros::Duration(footstep_plan_[i][4]);

    footstep_plan_msg.footsteps.push_back(footstep);
  }

  footstep_plan_pub_.publish(footstep_plan_msg);
}

void LocalFootstepPlanner::publishViz() {

  visualization_msgs::Marker points;
  points.header.frame_id = map_frame_;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  for (int i=0;i<footstep_plan_.size(); ++i) {
    geometry_msgs::Point p;

    p.x = footstep_plan_[i][1];
    p.y = footstep_plan_[i][2];
    p.z = 0.1;

    points.points.push_back(p);
  }

  footstep_plan_viz_pub_.publish(points);
}

void LocalFootstepPlanner::spin() {
  ros::Rate r(update_rate_);

  // Spin until body plan message has been received and processed
  boost::shared_ptr<spirit_msgs::BodyPlan const> plan_ptr;
  while((plan_ptr == nullptr) && (ros::ok()))
  {
    plan_ptr = ros::topic::waitForMessage<spirit_msgs::BodyPlan>("/body_plan", nh_);
    ros::spinOnce();
    r.sleep();
  }

  updatePlan();

  while (ros::ok()) {
    ROS_INFO("In spin, updating at %4.1f Hz", update_rate_);
    publishPlan();
    publishViz();
    ros::spinOnce();
    r.sleep();
  }
}