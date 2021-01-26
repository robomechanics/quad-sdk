#include "mpc_controller/mpc_controller.h"

MPCController::MPCController(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
    // extra comment
  std::string state_estimate_topic, control_input_topic,footstep_plan_topic,body_plan_topic, discrete_body_plan_topic;
  nh.param<std::string>("topics/state_estimate", state_estimate_topic, "/state_estimate");
  nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");
  nh.param<std::string>("topics/footstep_plan", footstep_plan_topic, "/footstep_plan");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");
  nh.param<std::string>("map_frame", map_frame_, "/map");

  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);


  // Setup pubs and subs
  state_estimate_sub_ = nh_.subscribe(state_estimate_topic,1,&MPCController::stateEstimateCallback, this);
  footstep_plan_sub_ = nh_.subscribe(footstep_plan_topic,1,&MPCController::footstepPlanCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&MPCController::bodyPlanCallback, this);
  discrete_body_plan_sub_ = nh_.subscribe(discrete_body_plan_topic,1,&MPCController::discreteBodyPlanCallback, this);
  control_input_pub_ = nh_.advertise<spirit_msgs::ControlInput>(control_input_topic,1);
}

void MPCController::stateEstimateCallback(const spirit_msgs::StateEstimate::ConstPtr& msg) {
  // ROS_INFO("In stateEstimateCallback");
}

void MPCController::footstepPlanCallback(const spirit_msgs::FootstepPlan::ConstPtr& msg) {
  // ROS_INFO("In footstepPlanCallback");
}

void MPCController::bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  // ROS_INFO("In bodyPlanCallback");
}

void MPCController::discreteBodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg) {
  // ROS_INFO("In discreteBodyPlanCallback");
}

void MPCController::publishControlInput() {
  // ROS_INFO("In ControlInput");
  spirit_msgs::ControlInput msg;

  msg.header.stamp = ros::Time::now();
  control_input_pub_.publish(msg);
}

void MPCController::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Publish control input data
    publishControlInput();
    ros::spinOnce();
    r.sleep();
  }
}