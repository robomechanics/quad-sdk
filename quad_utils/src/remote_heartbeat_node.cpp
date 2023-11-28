#include <ros/ros.h>
<<<<<<< HEAD

=======
<<<<<<< HEAD:quad_utils/src/remote_heartbeat_node.cpp

=======
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_utils/src/remote_heartbeat_node.cpp
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18
#include "quad_utils/remote_heartbeat.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "remote_heartbeat_node");
  ros::NodeHandle nh;

  RemoteHeartbeat remote_heartbeat(nh);
  remote_heartbeat.spin();

  return 0;
}
