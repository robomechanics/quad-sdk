#include "local_footstep_planner/local_footstep_planner.h"
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
    std::vector<double> s(9);
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

std::vector<double> LocalFootstepPlanner::interpolateMat(std::vector<double> input_vec, std::vector<std::vector<double>> input_mat, double query_point) {

  // Check bounds, throw an error if invalid since this shouldn't ever happen
  if ((query_point < input_vec.front()) || (query_point > input_vec.back())){
    throw std::runtime_error("Tried to interpolate out of bounds");
  }

  // Declare variables for interpolating between, both for input and output data
  double t1, t2;
  std::vector<double> y1, y2, interp_data;

  // Find the correct values to interpolate between
  int idx=0;
  for(int i=0;i<input_vec.size();i++)
  {
      if(input_vec[i]<=query_point && query_point<input_vec[i+1])
      {
        t1 = input_vec[i];
        t2 = input_vec[i+1];
        y1 = input_mat[i];
        y2 = input_mat[i+1]; 
        break;
      }
  }

  // Apply linear interpolation for each element in the vector
  for (int i = 0; i<input_mat.front().size(); i++) {
    double result = y1[i] + (y2[i]-y1[i])/(t2-t1)*(query_point-t1);
    interp_data.push_back(result);
  }
  
  return interp_data;
}

double LocalFootstepPlanner::interpolateVec(std::vector<double> input_vec, std::vector<double> input_mat, double query_point){

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

  // Loop through each gait cycle
  for (int i = 0; i < num_cycles; i++) {
    
    // Compute the initial time for this cycle
    double t_cycle = i*period;

    // Loop through each foot
    for (int j=0; j<num_feet; j++) {
      FootstepState footstep(5);

      // Compute the touchdown and midstance times
      double t_touchdown = t_cycle + t_offsets[j];
      double t_midstance = t_cycle + t_offsets[j] + 0.5*t_s[j];

      BodyState s_touchdown = interpolateMat(t_plan_, body_plan_, t_touchdown);
      BodyState s_midstance = interpolateMat(t_plan_, body_plan_, t_midstance);

      // Compute the body and hip positions and velocities
      double x_body = s_touchdown[0];
      double y_body = s_touchdown[1];
      double yaw = s_touchdown[8];
      double x_hip = x_body + x_offsets[j]*cos(yaw) - y_offsets[j]*sin(yaw);
      double y_hip = y_body + x_offsets[j]*sin(yaw) + y_offsets[j]*cos(yaw);
      double dx_body = s_midstance[3];
      double dy_body = s_midstance[4];

      // Load the data into the footstep array and push into the plan
      footstep[0] = j;
      footstep[1] = x_hip + 0.5*t_s[j]*dx_body;
      footstep[2] = y_hip + 0.5*t_s[j]*dy_body;
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