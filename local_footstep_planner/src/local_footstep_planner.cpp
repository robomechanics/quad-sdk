#include "local_footstep_planner/local_footstep_planner.h"
#include <tf/tf.h>

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

  // Specify the number of feet and their offsets from the COM
  double num_feet = 4;
  double x_offsets[4] = {0.3, -0.3, 0.3, -0.3};
  double y_offsets[4] = {0.2, 0.2, -0.2, -0.2};

  // For now, loop through every n states and calculate footsteps
  for (int i=0; i <t_plan_.size(); i+=4) {

    // Compute trig
    double sy = sin(body_plan_[i][8]);
    double cy = cos(body_plan_[i][8]);

    // Loop through each foot
    for (int j=0; j<num_feet; j++) {

      // Create a footstep to add to the plan
      FootstepState footstep;

      // Load in foot index, x & y location, touchdown time, and stance time
      footstep[0] = j;
      footstep[1] = body_plan_[i][0] + x_offsets[j]*cy - y_offsets[j]*sy;
      footstep[2] = body_plan_[i][1] + x_offsets[j]*sy + y_offsets[j]*cy;
      footstep[3] = t_plan_[i];
      footstep[4] = 0.2;

      // Add to the plan
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