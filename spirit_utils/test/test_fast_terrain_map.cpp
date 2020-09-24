#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/fast_terrain_map.h"

TEST(RVizInterfaceTest, testTrue) {
  ros::NodeHandle nh;
  FastTerrainMap fast_terrain_map;
  EXPECT_EQ(1 + 1, 2);
}