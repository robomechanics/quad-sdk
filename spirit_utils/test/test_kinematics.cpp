#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/kinematics.h"
#include "spirit_utils/foot_jacobians.h"

using namespace spirit_utils;

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testFKIKFeasibleConfigurations) {
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

TEST(KinematicsTest, testFKIKInfeasibleConfigurations) {
  ros::NodeHandle nh;
 
  SpiritKinematics spirit;

  // Set up problem variables
  Eigen::Vector3d body_pos = {0,0,0};
  Eigen::Vector3d body_rpy = {0,0,0};
  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d foot_pos_world_test;
  Eigen::Vector3d joint_state_test;

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

TEST(KinematicsTest, testBodyToFootFK) {
  ros::NodeHandle nh;
 
  // Declare kinematics object
  SpiritKinematics spirit;

  // Set up problem variables
  Eigen::Matrix4d g_world_foot;
  Eigen::Matrix4d g_body_foot;
  Eigen::Vector3d foot_pos_body;

  Eigen::Matrix4d g_body_foot_test;
  Eigen::Vector3d foot_pos_body_test;

  double pos_min = -1.0;
  double pos_max = 1.0;
  double roll_min = -M_PI;
  double roll_max = M_PI;
  double pitch_min = -0.5*M_PI;
  double pitch_max = 0.5*M_PI;
  double yaw_min = -M_PI;
  double yaw_max = M_PI;

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

    // Generate valid body orientations
    Eigen::Vector3d body_pos = {(pos_max - pos_min)*rand()/RAND_MAX + pos_min ,
      (pos_max - pos_min)*rand()/RAND_MAX + pos_min,
      (pos_max - pos_min)*rand()/RAND_MAX + pos_min};

    Eigen::Vector3d body_rpy = {(pos_max - roll_min)*rand()/RAND_MAX + roll_min,
      (pitch_max - pitch_min)*rand()/RAND_MAX + pitch_min,
      (yaw_max - yaw_min)*rand()/RAND_MAX + yaw_min};

    Eigen::Matrix4d g_world_body = spirit.createAffineMatrix(body_pos, body_rpy);

    for (int leg_index = 0; leg_index < 4; leg_index++) {

      // Compute the foot position in world frame with FK then tranform into body frame
      spirit.legFK(leg_index, body_pos, body_rpy, joint_state, g_world_foot);
      spirit.transformWorldToBody(body_pos, body_rpy, g_world_foot, g_body_foot);
      foot_pos_body = g_body_foot.block<3,1>(0,3);

      // Compute foot positions directly from the body frame
      spirit.bodyToFootFK(leg_index,joint_state,g_body_foot_test);
      spirit.bodyToFootFK(leg_index,joint_state,foot_pos_body_test);

      // Check the answers
      EXPECT_TRUE(foot_pos_body_test.isApprox(foot_pos_body));
      EXPECT_TRUE(g_body_foot_test.isApprox(g_body_foot));
    }
  }
}

TEST(KinematicsTest, testFootJacobians) {

  double pi = 3.14159;
  double q00 =pi/4,q01 =pi/4,q02=pi/2,q10 =pi/4,q11 =pi/4,q12=pi/2,q20 =-pi/4,q21 =pi/4,q22=pi/2,q30 =-pi/4,q31 =pi/4,q32=pi/2;
  double roll =0,pitch =0,yaw =0,x =0,y =-1,z =0;
  Eigen::VectorXd states(18);
  states << q00, q01, q02, q10, q11, q12, q20, q21, q22, q30, q31, q32, x, y, z, roll, pitch, yaw;

  const int num_feet = 4;
  Eigen::MatrixXd foot_jacobian(3*num_feet,states.size());
  spirit_utils::getJacobian(states,foot_jacobian);

  // Call the foot jacobian calculations
  for (int i = 0; i < num_feet; i++) {
    Eigen::MatrixXd foot_jacobian(3,3);
    spirit_utils::getFootJacobian(i,states,foot_jacobian);
  }
  
  EXPECT_EQ(1 + 1, 2);
}

