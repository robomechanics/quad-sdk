#include <gtest/gtest.h>
#include <ros/ros.h>

#include "quad_utils/terrain_map_publisher.h"

TEST(TerrainMapPublisherTest, testTrue) {
  ros::NodeHandle nh;
  TerrainMapPublisher terrain_map_publisher(nh);
  EXPECT_EQ(1 + 1, 2);
}
