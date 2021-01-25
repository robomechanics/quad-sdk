#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/trajectory_visualization_publisher.h"

TEST(TrajectoryVisualizationPublisherTest, testTrue) {
  ros::NodeHandle nh;
  TrajectoryVisualizationPublisher trajectory_visualization_publisher(nh);
  EXPECT_EQ(1 + 1, 2);
}