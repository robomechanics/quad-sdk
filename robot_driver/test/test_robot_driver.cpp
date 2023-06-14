#include <gtest/gtest.h>
#include <ros/ros.h>

#include "robot_driver/controllers/grf_pid_controller.h"
#include "robot_driver/controllers/inverse_dynamics_controller.h"
#include "robot_driver/controllers/joint_controller.h"
#include "robot_driver/robot_driver.h"

int my_argc;
char** my_argv;

TEST(LegController, testConstructorRobotDriver) {
  ros::NodeHandle nh;
  RobotDriver robot_driver(nh, my_argc, my_argv);
  EXPECT_EQ(1 + 1, 2);
}

TEST(LegController, testConstructorInverseDynamicsController) {
  InverseDynamicsController inverse_dynamics_controller;
  EXPECT_EQ(1 + 1, 2);
}

TEST(JointController, testConstructorJointController) {
  JointController joint_controller;
  EXPECT_EQ(1 + 1, 2);
}

TEST(GrfPidController, testConstructorGrfPidController) {
  GrfPidController grf_pid_controller;
  EXPECT_EQ(1 + 1, 2);
}

TEST(EKFEstimator, testEKFEstimator){
  ros::NodeHandle nh;
  EKFEstimator EKFEstimator;
  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  my_argc = argc;
  my_argv = argv;
  ros::init(argc, argv, "robot_driver_test");

  return RUN_ALL_TESTS();
}
