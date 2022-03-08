#include "quad_utils/robot_proxy.h"

RobotProxy::RobotProxy(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparams from parameter server
  std::string control_input_topic, joint_encoder_topic, imu_topic, twist_topic,
      mocap_topic;
  nh.param<std::string>("topics/control_input", control_input_topic,
                        "/control_input");
  nh.param<std::string>("topics/joint_encoder", joint_encoder_topic,
                        "/joint_encoder");
  nh.param<std::string>("topics/imu", imu_topic, "/imu");
  nh.param<std::string>("topics/vel", twist_topic, "/vel");
  nh.param<std::string>("topics/mocap", mocap_topic,
                        "/mocap_optitrack/quad/pose");
  nh.param<double>("robot_proxy/update_rate", update_rate_, 2);

  // Setup pubs and subs
  control_input_sub_ =
      nh_.subscribe(control_input_topic, 1, &RobotProxy::grfArrayCallback, this,
                    ros::TransportHints().tcpNoDelay(true));
  joint_encoder_pub_ =
      nh_.advertise<sensor_msgs::JointState>(joint_encoder_topic, 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic, 1);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_topic, 1);
  mocap_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(mocap_topic, 1);
}

void RobotProxy::grfArrayCallback(const quad_msgs::GRFArray::ConstPtr& msg) {}

void RobotProxy::publishJointEncoders() {
  sensor_msgs::JointState msg;

  double pi = 3.14159;

  std::vector<std::string> joint_names = {"8",  "0", "1", "9",  "2", "3",
                                          "10", "4", "5", "11", "6", "7"};
  std::vector<double> joint_pos = {0,         0.25 * pi, 0.5 * pi,  0,
                                   0.25 * pi, 0.5 * pi,  0,         0.25 * pi,
                                   0.5 * pi,  0,         0.25 * pi, 0.5 * pi};
  std::vector<double> joint_vel(12, 0);
  std::vector<double> joint_effort(12, 0);

  msg.name = joint_names;
  msg.position = joint_pos;
  msg.velocity = joint_vel;
  msg.effort = joint_effort;
  msg.header.stamp = ros::Time::now();
  joint_encoder_pub_.publish(msg);
}

void RobotProxy::publishImu() {
  sensor_msgs::Imu msg;
  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = 0;
  msg.orientation.w = 1;
  imu_pub_.publish(msg);

  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.twist.angular.x = 0;
  twist_msg.twist.angular.y = 0;
  twist_msg.twist.angular.z = 0;
  twist_pub_.publish(twist_msg);
}

void RobotProxy::publishMocap() {
  double pi = 3.14159;

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = 0;
  msg.pose.position.y = 0;
  msg.pose.position.z = 2 * (0.206 * sin(0.25 * pi));

  double t_now = ros::Time::now().toSec();
  double amplitude = -0.5;
  double period = 10;
  // msg.pose.position.x = amplitude*sin(2*3.14159*(t_now - t_init_)/period);

  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = 0;
  msg.pose.orientation.w = 1;
  mocap_pub_.publish(msg);
}

void RobotProxy::spin() {
  ros::Rate r(update_rate_);
  t_init_ = ros::Time::now().toSec();
  while (ros::ok()) {
    // Publish sensor data
    publishJointEncoders();
    publishImu();
    publishMocap();

    // Enforce update rate
    ros::spinOnce();
    r.sleep();
  }
}
