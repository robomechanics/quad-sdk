#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/kinematics.h"

using namespace spirit_utils;

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testFeasibleConfigurations) {
  ros::NodeHandle nh;
 
  // Declare kinematics object
  SpiritKinematics spirit;

  // Set up problem variables
  Eigen::Vector3d body_pos = {0,0,0};
  Eigen::Vector3d body_rpy = {0,0,0};
  Eigen::Vector3d joint_state = {0.7,-0.3*M_PI,0*M_PI};
  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d joint_state_test;
  Eigen::Vector3d foot_pos_world_test;

  // Compute the kinematics
  int N = 10000;
  for (int config = 0; config < N; config++) {
    
    // Generate valid joint configurations
    Eigen::Vector3d joint_state =
      { (spirit.getJointUpperLimit(0) - spirit.getJointLowerLimit(0))*
        (double)rand()/RAND_MAX + spirit.getJointLowerLimit(0),
        (spirit.getJointUpperLimit(1) - spirit.getJointLowerLimit(1))*
        (double)rand()/RAND_MAX + spirit.getJointLowerLimit(1),
        (spirit.getJointUpperLimit(2) - spirit.getJointLowerLimit(2))*
        (double)rand()/RAND_MAX + spirit.getJointLowerLimit(2) };

    for (int i = 0; i < 4; i++) {
      int leg_index = i;

      // Compute foot positions in this configuration
      spirit.legFK(leg_index,body_pos,body_rpy,joint_state,foot_pos_world);

      // Run IK to compute corresponding joint angles, then back through FK
      // This ensures that we are enforcing a hip-above-knee configuration if
      // otherwise ambiguous.
      spirit.legIK(leg_index,body_pos,body_rpy,foot_pos_world,joint_state_test);
      spirit.legFK(leg_index,body_pos,body_rpy,joint_state_test,
        foot_pos_world_test);

      // Check the answers
      Eigen::Vector3d error = (foot_pos_world - foot_pos_world_test);
      EXPECT_TRUE(error.norm() <= kinematics_tol);
    }
  }
}

TEST(KinematicsTest, testInfeasibleConfigurations) {
  ros::NodeHandle nh;
 
  SpiritKinematics spirit;

  // Set up problem variables
  Eigen::Vector3d body_pos = {0,0,0};
  Eigen::Vector3d body_rpy = {0,0,0};
  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d foot_pos_world_test;
  Eigen::Vector3d joint_state_test;

  // Test foot locations too close to hip
  for (int i = 0; i < 4; i++) {
    int leg_index = i;

    // Compute foot position for desired offset from leg base
    Eigen::Vector3d shoulder_pos;
    spirit.legBaseFK(leg_index, body_pos, body_rpy, shoulder_pos);
    Eigen::Vector3d foot_offset = {0,0.5*spirit.getLinkLength(i,0),
      0.5*spirit.getLinkLength(i,0)};
    foot_pos_world = shoulder_pos + foot_offset;

    // Compute IK, then check with FK foot position
    spirit.legIK(leg_index,body_pos,body_rpy,foot_pos_world,joint_state_test);
    spirit.legFK(leg_index,body_pos,body_rpy,joint_state_test,
         foot_pos_world_test);

    // Get actual closest foot position (in direction of desired foot offset but
    // respecting the length of the abad link)
    Eigen::Vector3d min_foot_offset = foot_offset;
    min_foot_offset.tail<2>().normalize();
    min_foot_offset.tail<2>() = abs(spirit.getLinkLength(i,0)) *
      min_foot_offset.tail<2>();

    // Compute the error from closest position and ensure it is zero
    Eigen::Vector3d error = (min_foot_offset - 
      (foot_pos_world_test - shoulder_pos));
    EXPECT_TRUE(error.norm() <= kinematics_tol);
  }

  // Define arbitrary maximum foot offset for IK testing
  double max_offset = abs(spirit.getJointLowerLimit(0)) + 
    spirit.getJointLowerLimit(1) + spirit.getJointLowerLimit(2);

  // Test random foot positions to make sure nothing breaks
  int N = 10000;
  for (int config = 0; config < N; config++) {
    
    // Generate random foot offset
    Eigen::Vector3d foot_offset =
      { 2*max_offset*(double)rand()/RAND_MAX - max_offset,
        2*max_offset*(double)rand()/RAND_MAX - max_offset,
        2*max_offset*(double)rand()/RAND_MAX - max_offset};

    for (int i = 0; i < 4; i++) {
      int leg_index = i;

      // Transform foot offset into world frame
      Eigen::Vector3d shoulder_pos;
      spirit.legBaseFK(leg_index, body_pos, body_rpy, shoulder_pos);
      foot_pos_world = shoulder_pos + foot_offset;

      // Run IK and make sure there aren't any errors
      spirit.legIK(leg_index,body_pos,body_rpy,foot_pos_world,joint_state_test);

      // To do: Check these solutions and make sure they are what we want
    }
  }

  EXPECT_EQ(1+1,2);
}