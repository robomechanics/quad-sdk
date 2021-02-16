#include "inverse_dynamics/inverse_dynamics.h"

inverseDynamics::inverseDynamics(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
  std::string control_input_topic, state_estimate_topic, swing_leg_plan_topic, foot_step_plan_topic, leg_command_array_topic; 
  // nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");

  // Setup pubs and subs
  control_input_sub_ = nh_.subscribe(control_input_topic,1,&inverseDynamics::controlInputCallback, this);
  state_estimate_sub_= nh_.subscribe(state_estimate_topic,1,&inverseDynamics::stateEstimateCallback, this);
  swing_leg_plan_sub_= nh_.subscribe(swing_leg_plan_topic,1,&inverseDynamics::swingLegPlanCallback, this);
  foot_step_plan_sub_= nh_.subscribe(foot_step_plan_topic,1,&inverseDynamics::footStepPlanCallback, this);
  leg_command_array_pub_ = nh_.advertise<spirit_msgs::LegCommandArray>(leg_command_array_topic,1);
}

void inverseDynamics::controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
  last_control_input_msg_ = msg;
}
void inverseDynamics::stateEstimateCallback(const spirit_msgs::StateEstimate::ConstPtr& msg) {
  // ROS_INFO("In stateEstimateCallback");
  last_state_estimate_msg_ = msg;
}
void inverseDynamics::swingLegPlanCallback(const spirit_msgs::SwingLegPlan::ConstPtr& msg) {
  // ROS_INFO("In swingLegPlanCallback");
  last_swing_leg_plan_msg_ = msg;
}
void inverseDynamics::footStepPlanCallback(const spirit_msgs::FootStepPlan::ConstPtr& msg) {
  // ROS_INFO("In footSteplsPlanCallback");
  last_foot_step_plan_msg_ = msg;
}

void inverseDynamics::publishLegCommandArray() {
  // ROS_INFO("In inverseDynamics");
  spirit_msgs::LegCommandArray msg;
  // Pack 4 LegCommands in the LegCommandArray
  // Pack 3 MotorCommands in a LegCommand
  msg.header.stamp = ros::Time::now();
  leg_command_array_pub_.publish(msg);
}
void inverseDynamics::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Collect new messages on subscriber topics
    ros::spinOnce();

    // Publish control input data
    publishLegCommandArray();

    // Enforce update rate
    r.sleep();
  }
}