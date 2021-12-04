#include <ros/ros.h>
#include <gtest/gtest.h>

#include "quad_utils/ground_truth_publisher.h"

TEST(GroundTruthPublisherTest, testTrue) {
  ros::NodeHandle nh;
  GroundTruthPublisher ground_truth_publisher(nh);
  EXPECT_EQ(1 + 1, 2);
}