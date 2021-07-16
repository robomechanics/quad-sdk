#include "spirit_utils/remote_heartbeat.h"

RemoteHeartbeat::RemoteHeartbeat(ros::NodeHandle nh) {
  nh_ = nh;

  // Load rosparam from parameter server
  std::string heartbeat_topic;
  spirit_utils::loadROSParam(nh_, "topics/remote_heartbeat", heartbeat_topic);
  spirit_utils::loadROSParam(nh_, "remote_heartbeat/update_rate", update_rate_);

  // Setup pub
  heartbeat_pub_ = nh_.advertise<std_msgs::Header>(heartbeat_topic,1);
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
