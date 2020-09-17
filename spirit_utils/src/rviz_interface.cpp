#include "spirit_utils/rviz_interface.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

RVizInterface::RVizInterface(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string body_plan_topic, body_plan_viz_topic, footstep_plan_topic, footstep_plan_viz_topic, map_frame_;
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/visualization/body_plan", body_plan_viz_topic, "/visualization/body_plan");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<std::string>("topics/visualization/footstep_plan", footstep_plan_viz_topic, "/visualization/footstep_plan_viz");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("visualization/update_rate", update_rate_, 10); // add a param for your package instead of using the estimator one

  // Setup pubs and subs here
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&RVizInterface::bodyPlanCallback, this);
  footstep_plan_sub_ = nh_.subscribe(footstep_plan_topic,1,&RVizInterface::footstepPlanCallback, this);
  body_plan_viz_pub_ = nh_.advertise<nav_msgs::Path>(body_plan_viz_topic,1);
  footstep_plan_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(footstep_plan_viz_topic,1);
}

void RVizInterface::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  nav_msgs::Path body_plan_viz;

  body_plan_viz.header = msg->header;

  // Loop through the message to get the state info and add to private vector
  int length = msg->states.size();
  for (int i=0; i < length; i++) {

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->states[i].header;
    pose_stamped.pose = msg->states[i].pose.pose;

    body_plan_viz.poses.push_back(pose_stamped);
  }

  body_plan_viz_pub_.publish(body_plan_viz);
}

void RVizInterface::footstepPlanCallback(const spirit_msgs::FootstepPlan::ConstPtr& msg) {
  visualization_msgs::Marker points;
  points.header = msg->header;
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

  int length = msg->footsteps.size();
  for (int i=0;i<length; ++i) {
    geometry_msgs::Point p;

    p = msg->footsteps[i].position;
    p.z = 0.1;

    points.points.push_back(p);
  }

  footstep_plan_viz_pub_.publish(points);
}

void RVizInterface::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics

    ros::spinOnce();
    // Enforce update rate
    r.sleep();
  }
}