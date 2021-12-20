#include <ros/ros.h>
#include "quad_utils/remote_heartbeat.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "remote_heartbeat_node");
  ros::NodeHandle nh;

  RemoteHeartbeat remote_heartbeat(nh);
  remote_heartbeat.spin();
  
  return 0;
}
