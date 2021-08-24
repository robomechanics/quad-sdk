#include <ros/ros.h>

#include "spirit_utils/mesh_to_grid_map_converter.hpp"

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "mesh_to_grid_map");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Creating the object to do the work.
  mesh_to_grid_map::MeshToGridMapConverter mesh_to_grid_map_converter(
      nh, private_nh);

  ros::spin();
  return 0;
}