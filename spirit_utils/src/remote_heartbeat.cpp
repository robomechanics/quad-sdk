#include "spirit_utils/remote_heartbeat.h"

RemoteHeartbeat::RemoteHeartbeat(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparam from parameter server
  std::string heartbeat_topic, leg_control_topic;
  spirit_utils::loadROSParam(nh_, "topics/remote_heartbeat", heartbeat_topic);
  spirit_utils::loadROSParam(nh_, "topics/control/joint_command",leg_control_topic);
  spirit_utils::loadROSParam(nh_, "remote_heartbeat/update_rate", update_rate_);
  spirit_utils::loadROSParam(nh_, "remote_heartbeat/latency_threshold_warn",
    latency_threshold_warn_);
  spirit_utils::loadROSParam(nh_, "remote_heartbeat/latency_threshold_error",
    latency_threshold_error_);

  // Setup pub
  heartbeat_pub_ = nh_.advertise<std_msgs::Header>(heartbeat_topic,1);
  leg_control_sub_ = nh_.subscribe(leg_control_topic,1,&RemoteHeartbeat::legControlCallback, this);
}

void RemoteHeartbeat::legControlCallback(
  const spirit_msgs::LegCommandArray::ConstPtr& msg)
{
  // Get the current time and compare to the message time
  double last_leg_command_time = msg->header.stamp.toSec();
  double t_now = ros::Time::now().toSec();
  double t_latency = t_now - last_leg_command_time;

  if (abs(t_latency) >= latency_threshold_warn_) {
    ROS_WARN_THROTTLE(1.0,"Latency = %6.4fs which exceeds the warning threshold of %6.4fs\n",
      t_latency, latency_threshold_warn_);
  }

  if (abs(t_latency) >= latency_threshold_error_) {
    ROS_ERROR("Latency = %6.4fs which exceeds the maximum threshold of %6.4fs, "
      "killing remote heartbeat\n", t_latency, latency_threshold_error_);
  }
  
}

void RemoteHeartbeat::spin() {
  ros::Rate r(update_rate_);
  while (ros::ok()) {

    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    heartbeat_pub_.publish(msg);
    
    // Enforce update rate
    ros::spinOnce();
    r.sleep();
  }
}
