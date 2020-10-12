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

void RobotInterface::controlInputCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // ROS_INFO("In controlInputCallback");
}

void RobotInterface::publishJointEncoders() {
  // ROS_INFO("In publishJointEncoders");
}

void RobotInterface::publishImu() {
  // ROS_INFO("In publishImu");
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