#include "inverse_dynamics/inverse_dynamics.h"

inverseDynamics::inverseDynamics(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
    // extra comment
  std::string  control_input_topic,state_estimate_topic,swing_leg_plan_topic motor_command_topic,foot_step_plan_topic; 
  // nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");


  // nh.param<double>("mpc_controller/update_rate", update_rate_, 100);


  // Setup pubs and subs

  control_input_sub_ = nh_.subscribe(control_input_topic,1,&inverseDynamics::controlInputCallback, this);
  state_estimate_sub_= nh_.subscribe(state_estimate_topic,1,&inverseDynamics::stateEstimateCallback, this);
  swing_leg_plan_sub_= nh_.subscribe(swing_leg_plan_topic,1,&inverseDynamics::swingLegPlanCallback, this);
  foot_step_plan_sub_= nh_.subscribe(foot_step_plan_topic,1,&inverseDynamics::footStepPlanCallback, this);
  motor_command_pub_ = nh_.advertise<spirit_msgs:MotorCommand:>(motor_command_topic,1);
}

void inverseDynamics::controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
}
void inverseDynamics::stateEstimateCallback(const spirit_msgs::StateEstimate::ConstPtr& msg) {
  // ROS_INFO("In stateEstimateCallback");
}
void inverseDynamics::swingLegPlanCallback(const spirit_msgs::SwingLegPlan::ConstPtr& msg) {
  // ROS_INFO("In swingLegPlanCallback");
}
void inverseDynamics::footStepPlanCallback(const spirit_msgs::FootStepPlan::ConstPtr& msg) {
  // ROS_INFO("In footSteplsPlanCallback");
}

  void inverseDynamics::publishMotorCommand(){
  // ROS_INFO("In inverseDynamics");
  spirit_msgs::MotorCommand msg;
  msg.header.stamp = ros::Time::now();
  control_input_pub_.publish(msg);
}
void inverseDynamics::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Publish control input data
    publishInverseDynamics();
    ros::spinOnce();
    r.sleep();
  }
}