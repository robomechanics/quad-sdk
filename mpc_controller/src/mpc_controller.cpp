#include "mpc_controller/mpc_controller.h"

MPCController::MPCController(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string robot_state_topic, control_input_topic,foot_plan_discrete_topic,body_plan_topic, discrete_body_plan_topic;
  nh.param<std::string>("topics/state/ground_truth", robot_state_topic, "/state/ground_truth");
  nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");
  nh.param<std::string>("topics/foot_plan_discrete", foot_plan_discrete_topic, "/foot_plan_discrete");
  nh.param<std::string>("topics/body_plan", body_plan_topic, "/body_plan");
  nh.param<std::string>("topics/discrete_body_plan", discrete_body_plan_topic, "/discrete_body_plan");
  nh.param<std::string>("map_frame", map_frame_, "/map");

  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);

  // Setup pubs and subs
  robot_state_sub_ = nh_.subscribe(robot_state_topic,1,&MPCController::robotStateCallback, this);
  footstep_plan_sub_ = nh_.subscribe(foot_plan_discrete_topic,1,&MPCController::footPlanDiscreteCallback, this);
  body_plan_sub_ = nh_.subscribe(body_plan_topic,1,&MPCController::bodyPlanCallback, this);
  discrete_body_plan_sub_ = nh_.subscribe(discrete_body_plan_topic,1,&MPCController::discreteBodyPlanCallback, this);
  control_input_pub_ = nh_.advertise<spirit_msgs::ControlInput>(control_input_topic,1);
}

void MPCController::robotStateCallback(const spirit_msgs::RobotState::ConstPtr& msg) {
  // ROS_INFO("In robotStateCallback");
}

void MPCController::footPlanDiscreteCallback(const spirit_msgs::MultiFootPlanDiscrete::ConstPtr& msg) {
  // ROS_INFO("In footPlanDiscreteCallback");
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