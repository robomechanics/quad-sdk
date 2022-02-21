#include <gtest/gtest.h>
#include <ros/ros.h>

#include "ekf_estimator/ekf_estimator.h"

TEST(EKFEstimator, testTrue) {
  ros::NodeHandle nh;
  EKFEstimator ekf_estimator(nh);
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ekf_estimator_tester");

  return RUN_ALL_TESTS();
}
