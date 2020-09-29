/*
 * image_to_gridmap_demo_node.cpp
 *
 *  Created on: May 04, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "spirit_utils/ImageToGridmapDemoCustom.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_gridmap_demo_custom_node");
  ros::NodeHandle nh;


  ImageToGridmapDemo imageToGridmapDemo(nh);

  ros::spin();
  return 0;
}
