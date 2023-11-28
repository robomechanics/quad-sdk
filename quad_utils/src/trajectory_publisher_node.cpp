#include <ros/ros.h>
<<<<<<< HEAD:quad_utils/src/trajectory_publisher_node.cpp

=======
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_utils/src/trajectory_publisher_node.cpp
#include "quad_utils/trajectory_publisher.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_publisher_node");
  ros::NodeHandle nh;

  TrajectoryPublisher trajectory_publisher(nh);
  trajectory_publisher.spin();

  return 0;
}
