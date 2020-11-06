#include "spirit_utils/robot_interface.h"

RobotInterface::RobotInterface(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string control_input_topic, joint_encoder_topic, imu_topic;
  nh.param<std::string>("topics/control_input", control_input_topic, "/control_input");
  nh.param<std::string>("topics/joint_encoder", joint_encoder_topic, "/joint_encoder");
  nh.param<std::string>("topics/imu", imu_topic, "/imu");
  nh.param<double>("robot_interface/update_rate", update_rate_, 2);


  // Setup pubs and subs
  control_input_sub_ = nh_.subscribe(control_input_topic,1,&RobotInterface::controlInputCallback, this);
  joint_encoder_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_encoder_topic,1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic,1);
}

void RobotInterface::controlInputCallback(const spirit_msgs::ControlInput::ConstPtr& msg) {
}

void RobotInterface::publishJointEncoders() {
  sensor_msgs::JointState msg;

  std::vector<std::string> joint_names = {"8", "0", "1", "9","2", "3", "10", "4","5", "11", "6", "7"};
  std::vector<double> joint_pos (12,0);
  std::vector<double> joint_vel (12,0);
  std::vector<double> joint_effort (12,0);

  msg.name = joint_names;
  msg.position = joint_pos;
  msg.velocity = joint_vel;
  msg.effort = joint_effort;
  msg.header.stamp = ros::Time::now();
  joint_encoder_pub_.publish(msg);
}

void RobotInterface::publishImu() {
  sensor_msgs::Imu msg;
  imu_pub_.publish(msg);
}

void RobotInterface::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    // Publish sensor data
    publishJointEncoders();
    publishImu();
    
    // Enforce update rate
    ros::spinOnce();
    r.sleep();
  }
}
