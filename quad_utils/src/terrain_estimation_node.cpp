#include <ros/ros.h>

#include "quad_utils/terrain_estimation.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "terrain_estimation_node");
  ros::NodeHandle nh;

  TerrainEstimation terrain_estimation_node(nh);
  terrain_estimation_node.spin();

  return 0;
}
