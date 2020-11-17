#include "body_force_estimator/body_force_estimator.h"

BodyForceEstimator::BodyForceEstimator(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string state_estimate_topic, body_force_topic;
  nh.param<std::string>("topics/state_estimate", state_estimate_topic, "/state_estimate");
  nh.param<std::string>("topics/body_force", body_force_topic, "/body_force");
  nh.param<double>("body_force_estimator/update_rate", update_rate_, 200); // add a param for your package instead of using the estimator one

  // Setup pubs and subs
  state_estimate_sub_ = nh_.subscribe(state_estimate_topic,1,&BodyForceEstimator::stateEstimateCallback, this);
  body_force_pub_ = nh_.advertise<spirit_msgs::BodyForceEstimate>(body_force_topic,1);
}

void BodyForceEstimator::stateEstimateCallback(const spirit_msgs::StateEstimate::ConstPtr& msg) {
  // ROS_INFO("In stateEstimateCallback");
}

void BodyForceEstimator::publishBodyForce() {
  // ROS_INFO("In BodyForce");
  spirit_msgs::BodyForceEstimate msg;
  body_force_pub_.publish(msg);
}

void BodyForceEstimator::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Collect new messages on subscriber topics

    ros::spinOnce();
    // Enforce update rate
    r.sleep();
  }
}
