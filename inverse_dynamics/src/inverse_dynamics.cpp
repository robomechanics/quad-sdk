#include "inverse_dynamics/inverse_dynamics.h"

inverseDynamics::inverseDynamics(ros::NodeHandle nh) {
  nh.param<double>("mpc_controller/update_rate", update_rate_, 100);
	nh_ = nh;

    // Load rosparams from parameter server
    // extra comment
  std::string  control_input_topic, leg_torque_topic; // ADD TORQUE TOPIC????
  // nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");


  // nh.param<double>("mpc_controller/update_rate", update_rate_, 100);


  // Setup pubs and subs

  control_input_sub_ = nh_.subscribe(control_input_topic,1,&inverseDynamics::controlInputCallback, this);
  inverse_dynamics_pub_ = nh_.advertise<spirit_msgs:InverseDynamics:>(leg_torque_topic,1);
}

void inverseDynamics::controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg) {
  // ROS_INFO("In stateEstimateCallback");
}

  void inverseDynamics::publishInverseDynamics(){
  // ROS_INFO("In inverseDynamics");
  // spirit_msgs::ControlInput msg; Need a new message??

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