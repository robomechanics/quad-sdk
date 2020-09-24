#include "local_footstep_planner/local_footstep_planner.h"
#include <libInterpolate/Interpolate.hpp>
#include <tf/tf.h>
#include <chrono>

LocalFootstepPlanner::LocalFootstepPlanner(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, body_plan_topic, footstep_plan_topic;
  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("local_footstep_planner/update_rate", update_rate_, 1);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(terrain_map_topic,1,&LocalFootstepPlanner::terrainMapCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&LocalFootstepPlanner::bodyPlanCallback, this);
  footstep_plan_pub_ = nh_.advertise<spirit_msgs::FootstepPlan>(footstep_plan_topic,1);
}

void LocalFootstepPlanner::terrainMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  terrain_.loadDataFromGridMap(map);
}

void LocalFootstepPlanner::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  t_plan_.clear();
  body_plan_.clear();

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

  // Clear out the old footstep plan
  footstep_plan_.clear();

  // Define the gait sequence
  double period = 0.2;
  double t_offsets[4] = {0.0, 0.5*period, 0.5*period, 0.0};
  double t_s[4] = {0.5*period, 0.5*period, 0.5*period, 0.5*period};
  int num_cycles = t_plan_.back()/period;

  // Specify the number of feet and their offsets from the COM
  double num_feet = 4;
  double x_offsets[4] = {0.3, -0.3, 0.3, -0.3};
  double y_offsets[4] = {0.2, 0.2, -0.2, -0.2};

  // Create transposed body plan for easy interpolation
  std::vector<std::vector<double>> body_plan_transpose;
  for (int i=0; i<9; i++) {
    std::vector<double> v;
    for (auto s : body_plan_) {
      v.push_back(s[i]);
    }
    body_plan_transpose.push_back(v);
  }

  // Define interpolators for each variable (TODO: condense this)
  _1D::LinearInterpolator<double> interp_x;
  _1D::LinearInterpolator<double> interp_y;
  _1D::LinearInterpolator<double> interp_dx;
  _1D::LinearInterpolator<double> interp_dy;
  _1D::LinearInterpolator<double> interp_yaw;
  interp_x.setData(t_plan_,body_plan_transpose[0]);
  interp_y.setData(t_plan_,body_plan_transpose[1]);
  interp_dx.setData(t_plan_,body_plan_transpose[3]);
  interp_dy.setData(t_plan_,body_plan_transpose[4]);
  interp_yaw.setData(t_plan_,body_plan_transpose[8]);

  // Loop through each gait cycle
  for (int i = 0; i < num_cycles; i++) {
    
    // Compute the initial time for this cycle
    double t_cycle = i*period;

    // Loop through each foot
    for (int j=0; j<num_feet; j++) {
      FootstepState footstep;

      // Compute the touchdown and midstance times
      double t_touchdown = t_cycle + t_offsets[j];
      double t_midstance = t_cycle + t_offsets[j] + 0.5*t_s[j];

      // Compute the body and hip positions and velocities
      double x_body = interp_x(t_touchdown);
      double y_body = interp_y(t_touchdown);
      double x_hip = x_body + x_offsets[j]*cos(interp_yaw(t_touchdown)) 
        - y_offsets[j]*sin(interp_yaw(t_touchdown));
      double y_hip = y_body + x_offsets[j]*sin(interp_yaw(t_touchdown)) 
        + y_offsets[j]*cos(interp_yaw(t_touchdown));
      double dx_body = interp_x(t_midstance);
      double dy_body = interp_x(t_midstance);

      // Load the data into the footstep array and push into the plan
      footstep[0] = j;
      footstep[1] = x_hip + 0.5*t_s[j]*interp_dx(t_midstance);
      footstep[2] = y_hip + 0.5*t_s[j]*interp_dy(t_midstance);
      footstep[3] = t_touchdown;
      footstep[4] = t_s[j];

      footstep_plan_.push_back(footstep);
    }
  }
}

void LocalFootstepPlanner::publishPlan() {

  // Initialize FootstepPlan message
  spirit_msgs::FootstepPlan footstep_plan_msg;
  ros::Time timestamp = ros::Time::now();
  footstep_plan_msg.header.stamp = timestamp;
  footstep_plan_msg.header.frame_id = map_frame_;

  // Loop through the plan
  for (int i=0;i<footstep_plan_.size(); ++i) {

    // Initialize a footstep message and load the data
    spirit_msgs::Footstep footstep;

    footstep.index = footstep_plan_[i][0];
    footstep.position.x = footstep_plan_[i][1];
    footstep.position.y = footstep_plan_[i][2];
    footstep.position.z = terrain_.getGroundHeight(footstep.position.x,footstep.position.y);
    footstep.td = ros::Duration(footstep_plan_[i][3]);
    footstep.ts = ros::Duration(footstep_plan_[i][4]);

    footstep_plan_msg.footsteps.push_back(footstep);
  }

  // Publish the whole plan to the topic
  footstep_plan_pub_.publish(footstep_plan_msg);
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

  // Enter spin
  while (ros::ok()) {
    // ROS_INFO("In LocalFootstepPlanner spin, updating at %4.1f Hz", update_rate_);
    
    // Update the plan and publish it
    updatePlan();
    publishPlan();

    ros::spinOnce();
    r.sleep();
  }
}