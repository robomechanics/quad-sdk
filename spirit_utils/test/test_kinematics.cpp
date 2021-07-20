#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/kinematics.h"

using namespace spirit_utils;

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testDifferentialFKIK)
{
  // Declare kinematics object
  SpiritKinematics kinematics;

  for (size_t i = 0; i < 10; i++)
  {
    // Declare input and output RobotState object
    spirit_msgs::RobotState state, state_out;

    // Random velocities at origin
    Eigen::VectorXd body_state(12);
    body_state << 0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10 * (double)rand() / RAND_MAX - 5,
        10 * (double)rand() / RAND_MAX - 5,
        10 * (double)rand() / RAND_MAX - 5,
        3.14 * (double)rand() / RAND_MAX - 1.57,
        3.14 * (double)rand() / RAND_MAX - 1.57,
        3.14 * (double)rand() / RAND_MAX - 1.57;

    state.body = eigenToOdomMsg(body_state);

    state.joints.name = {"8", "0", "1", "9", "2", "3", "10", "4", "5", "11", "6", "7"};
    state.joints.position.clear();
    state.joints.velocity.clear();
    state.joints.effort.clear();

    for (int j = 0; j < 4; j++)
    {
      // Just some arbitary joints position
      state.joints.position.push_back(0.1);
      state.joints.position.push_back(0.2);
      state.joints.position.push_back(0.3);

      // Random joints velocity
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);

      // We don't need joints effort here
      state.joints.effort.push_back(0.0);
      state.joints.effort.push_back(0.0);
      state.joints.effort.push_back(0.0);
    }

    // Run FK get foot velocities and IK them back
    spirit_utils::fkRobotState(kinematics, state.body, state.joints, state.feet);
    spirit_utils::ikRobotState(kinematics, state.body, state.feet, state_out.joints);

    // Extract input joint velocities
    Eigen::VectorXd vel(12), vel_out(12);
    vectorToEigen(state.joints.velocity, vel);
    vectorToEigen(state_out.joints.velocity, vel_out);

    // Check the answers
    Eigen::VectorXd error = vel - vel_out;
    EXPECT_TRUE(error.norm() <= kinematics_tol);
  }
}

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
