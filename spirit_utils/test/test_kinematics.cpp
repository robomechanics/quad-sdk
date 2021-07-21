#include <ros/ros.h>
#include <gtest/gtest.h>

#include "spirit_utils/kinematics.h"
#include "spirit_utils/foot_jacobians.h"

using namespace spirit_utils;

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testDifferentialFKIK)
{
  // Declare kinematics object
  SpiritKinematics kinematics;

  for (size_t i = 0; i < 20; i++)
  {
    // Declare input and output RobotState object
    spirit_msgs::RobotState state, state_out;

    // Random velocities at origin
    Eigen::VectorXd body_state(12);
    body_state << (double)rand() / RAND_MAX - 0.5,
        (double)rand() / RAND_MAX - 0.5,
        (double)rand() / RAND_MAX - 0.5,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        1.5 * (double)rand() / RAND_MAX - 0.75,
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


TEST(KinematicsTest, testFootForces) {

  // Length parameters from URDF
  // TODO: load these from parameters rather than hard-coding
  Eigen::MatrixXd ls(4,3);
  ls << 0.2263, 0.07, 0.0, // abad from body
    0.0, 0.10098, 0.0, // hip from abad
    -0.206, 0.0, 0.0, // knee from hip
    0.206, 0.0, 0.0; // toe from knee

  double pi = 3.14159265359;
    
  // Define vectors for states, forces, and torques
  Eigen::VectorXd state_positions(18), forces(12), torques(18), torques_solution(18);

  // Compute jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  
  // Set up known solution problem 1 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i=0; i<3; i++){
    // move the CG around randomly -- it should not matter
    state_positions(12+i) = (double)rand()/RAND_MAX - 0.5;
  }
  forces = Eigen::VectorXd::Zero(12);
  forces(2) = 3.0; // front left toe Z
  forces(3) = 2.0; // back left toe X

  // Known solution
  torques_solution << 3.0 * ls(1,1),
    0.0,
    3.0 * -ls(3,0),
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    2.0,
    0.0,
    3.0,
    3.0 * (ls(0,1) + ls(1,1)),
    3.0 * -ls(0,0),
    -2.0 * (ls(0,1) + ls(1,1));

  // Compute joint torques
  spirit_utils::getJacobian(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  Eigen::VectorXd error = torques - torques_solution;
  Eigen::MatrixXd toPrint(18,2);
  toPrint << torques, torques_solution;
  std::cout << "Test 1:\n" << toPrint << std::endl
  << torques_solution.transpose() << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);

  // Set up known solution problem 2 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i=0; i<3; i++){
    // move the CG around randomly -- it should not matter
    state_positions(12+i) = (double)rand()/RAND_MAX - 0.5;
  }
  state_positions(17) = pi/2; // yaw 90 deg left
  state_positions(7) = pi/4; // front right hip 45 deg down
  state_positions(8) = pi/2; // front right knee 90 deg down
  forces = Eigen::VectorXd::Zero(12);
  forces(6) = 3.0; // front right toe X
  forces(8) = 5.0; // front right toe Z

  // Known solution
  torques_solution << 0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -5.0 * ls(1,1) - 3.0 * (-ls(2,0)*sin(pi/4) + ls(3,0)*sin(pi/4)),
    0.0,
    5.0 * -ls(3,0)*cos(pi/4),
    0.0,
    0.0,
    0.0,
    3.0,
    0.0,
    5.0,
    -5.0 * (ls(0,1) + ls(1,1)) - 3.0 * (-ls(2,0)*sin(pi/4) + ls(3,0)*sin(pi/4)),
    -5.0 * ls(0,0),
    -3.0 * ls(0,0);

  // Compute joint torques
  spirit_utils::getJacobian(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  error = torques - torques_solution;
  toPrint << torques, torques_solution;
  std::cout << "Test 2:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);

  // Set up known solution problem 3 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i=0; i<3; i++){
    // move the CG around randomly -- it should not matter
    state_positions(12+i) = (double)rand()/RAND_MAX - 0.5;
  }
  state_positions(15) = pi/2; // roll 90 deg right
  state_positions(17) = pi/2; // yaw 90 deg left
  forces = Eigen::VectorXd::Zero(12);
  forces(0) = 1.0; // front left toe X

  // Known solution
  torques_solution << ls(1,1),
    0.0,
    -ls(3,0),
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    ls(0,1) + ls(1,1),
    0.0,
    -ls(0,0);

  // Compute joint torques
  spirit_utils::getJacobian(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  error = torques - torques_solution;
  toPrint << torques, torques_solution;
  std::cout << "Test 2:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);


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

