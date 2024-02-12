#include "quad_utils/remote_heartbeat.h"

RemoteHeartbeat::RemoteHeartbeat(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparam from parameter server
  std::string remote_heartbeat_topic, robot_heartbeat_topic, leg_control_topic;
  quad_utils::loadROSParam(nh_, "/topics/heartbeat/remote",
                           remote_heartbeat_topic);
  quad_utils::loadROSParam(nh_, "topics/heartbeat/robot",
                           robot_heartbeat_topic);
  quad_utils::loadROSParam(nh_, "remote_heartbeat/robot_latency_threshold_warn",
                           robot_latency_threshold_warn_);
  quad_utils::loadROSParam(nh_,
                           "remote_heartbeat/robot_latency_threshold_error",
                           robot_latency_threshold_error_);
  quad_utils::loadROSParam(nh_, "remote_heartbeat/update_rate", update_rate_);

  // Setup pub
  remote_heartbeat_pub_ =
      nh_.advertise<std_msgs::Header>(remote_heartbeat_topic, 1);
  robot_heartbeat_sub_ = nh_.subscribe(
      robot_heartbeat_topic, 1, &RemoteHeartbeat::robotHeartbeatCallback, this);
}

void RemoteHeartbeat::robotHeartbeatCallback(
    const std_msgs::Header::ConstPtr& msg) {
  // Get the current time and compare to the message time
  double last_robot_heartbeat_time = msg->stamp.toSec();
  double t_now = ros::Time::now().toSec();
  double t_latency = t_now - last_robot_heartbeat_time;

  // ROS_INFO_THROTTLE(1.0,"Robot latency = %6.4fs", t_latency);

  if (abs(t_latency) >= robot_latency_threshold_warn_) {
    // ROS_WARN_THROTTLE(1.0,"Robot latency = %6.4fs which exceeds the warning
    // threshold of %6.4fs\n",
    //   t_latency, robot_latency_threshold_warn_);
  }

  if (abs(t_latency) >= robot_latency_threshold_error_) {
    // ROS_ERROR("Robot latency = %6.4fs which exceeds the maximum threshold of
    // %6.4fs, "
    //   "killing remote heartbeat\n", t_latency,
    //   robot_latency_threshold_error_);
    // throw std::runtime_error("Shutting down remote heartbeat");
  }
}

void RemoteHeartbeat::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    remote_heartbeat_pub_.publish(msg);

    // Enforce update rate
    ros::spinOnce();
    r.sleep();
  }
}