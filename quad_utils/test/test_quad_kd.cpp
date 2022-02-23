#include <gtest/gtest.h>
#include <ros/ros.h>

#include "quad_utils/quad_kd.h"
#include "quad_utils/ros_utils.h"

using namespace quad_utils;

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testDifferentialFKIK) {
  // Declare kinematics object
  QuadKD kinematics;

  for (size_t i = 0; i < 20; i++) {
    // Declare input and output RobotState object
    quad_msgs::RobotState state, state_out;

    // Random velocities at origin
    Eigen::VectorXd body_state(12);
    body_state << (double)rand() / RAND_MAX - 0.5,
        (double)rand() / RAND_MAX - 0.5, (double)rand() / RAND_MAX - 0.5,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        1.5 * (double)rand() / RAND_MAX - 0.75,
        10 * (double)rand() / RAND_MAX - 5, 10 * (double)rand() / RAND_MAX - 5,
        10 * (double)rand() / RAND_MAX - 5,
        3.14 * (double)rand() / RAND_MAX - 1.57,
        3.14 * (double)rand() / RAND_MAX - 1.57,
        3.14 * (double)rand() / RAND_MAX - 1.57;

    state.body = eigenToBodyStateMsg(body_state);

    state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                         "10", "4", "5", "11", "6", "7"};
    state.joints.position.clear();
    state.joints.velocity.clear();
    state.joints.effort.clear();

    for (int j = 0; j < 4; j++) {
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
    quad_utils::fkRobotState(kinematics, state.body, state.joints, state.feet);
    quad_utils::ikRobotState(kinematics, state.body, state.feet,
                             state_out.joints);

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
  // Declare kinematics object
  QuadKD kinematics;

  // Length parameters from URDF
  // TODO(yanhaoy): load these from parameters rather than hard-coding
  Eigen::MatrixXd ls(4, 3);
  ls << 0.2263, 0.07, 0.0,  // abad from body
      0.0, 0.10098, 0.0,    // hip from abad
      -0.206, 0.0, 0.0,     // knee from hip
      0.206, 0.0, 0.0;      // toe from knee

  double pi = 3.14159265359;

  // Define vectors for states, forces, and torques
  Eigen::VectorXd state_positions(18), forces(12), torques(18),
      torques_solution(18);

  // Compute jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);

  // Set up known solution problem 1 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i = 0; i < 3; i++) {
    // move the CG around randomly -- it should not matter
    state_positions(12 + i) = (double)rand() / RAND_MAX - 0.5;
  }
  forces = Eigen::VectorXd::Zero(12);
  forces(2) = 3.0;  // front left toe Z
  forces(3) = 2.0;  // back left toe X

  // Known solution
  torques_solution << 3.0 * ls(1, 1), 0.0, 3.0 * -ls(3, 0), 0.0, 0.0,
      0.0,                                          // leg 2
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 3.0,  // net forces
      3.0 * (ls(0, 1) + ls(1, 1)), 3.0 * -ls(0, 0),
      -2.0 * (ls(0, 1) + ls(1, 1));

  // Compute joint torques
  kinematics.getJacobianGenCoord(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  Eigen::VectorXd error = torques - torques_solution;
  Eigen::MatrixXd toPrint(18, 2);
  toPrint << torques, torques_solution;
  // std::cout << "Test 1:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);

  // Set up known solution problem 2 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i = 0; i < 3; i++) {
    // move the CG around randomly -- it should not matter
    state_positions(12 + i) = (double)rand() / RAND_MAX - 0.5;
  }
  state_positions(17) = pi / 2;  // yaw 90 deg left
  state_positions(7) = pi / 4;   // front right hip 45 deg down
  state_positions(8) = pi / 2;   // front right knee 90 deg down
  forces = Eigen::VectorXd::Zero(12);
  forces(6) = 3.0;  // front right toe X
  forces(8) = 5.0;  // front right toe Z

  // Known solution
  torques_solution << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // leg 2
      -5.0 * ls(1, 1) -
          3.0 * (-ls(2, 0) * sin(pi / 4) + ls(3, 0) * sin(pi / 4)),
      0.0, 5.0 * -ls(3, 0) * cos(pi / 4), 0.0, 0.0, 0.0, 3.0, 0.0,
      5.0,  // net forces
      -5.0 * (ls(0, 1) + ls(1, 1)) -
          3.0 * (-ls(2, 0) * sin(pi / 4) + ls(3, 0) * sin(pi / 4)),
      -5.0 * ls(0, 0), -3.0 * ls(0, 0);

  // Compute joint torques
  kinematics.getJacobianGenCoord(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  error = torques - torques_solution;
  toPrint << torques, torques_solution;
  // std::cout << "Test 2:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);

  // Set up known solution problem 3 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i = 0; i < 3; i++) {
    // move the CG around randomly -- it should not matter
    state_positions(12 + i) = (double)rand() / RAND_MAX - 0.5;
  }
  state_positions(15) = pi / 2;  // roll 90 deg right
  state_positions(17) = pi / 2;  // yaw 90 deg left
  forces = Eigen::VectorXd::Zero(12);
  forces(0) = 1.0;  // front left toe X

  // Known solution
  torques_solution << ls(1, 1), 0.0, -ls(3, 0), 0.0, 0.0, 0.0,  // leg 2
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,              // net forces
      ls(0, 1) + ls(1, 1), 0.0, -ls(0, 0);

  // Compute joint torques
  kinematics.getJacobianGenCoord(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  error = torques - torques_solution;
  toPrint << torques, torques_solution;
  // std::cout << "Test 3:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);

  // Set up known solution problem 4 ----------------------------------
  state_positions = Eigen::VectorXd::Zero(18);
  for (int i = 0; i < 3; i++) {
    // move the CG around randomly -- it should not matter
    state_positions(12 + i) = (double)rand() / RAND_MAX - 0.5;
  }
  state_positions(16) = pi / 4;  // pitch 45 deg down
  state_positions(17) = pi;      // yaw 180 deg
  state_positions(1) = pi / 4;   // front left hip 60 deg down
  state_positions(4) = -pi / 4;  // back left hip 45 deg up
  state_positions(7) = -pi / 4;  // front right hip 45 deg up
  state_positions(10) = pi / 4;  // back right hip 60 deg down
  forces = Eigen::VectorXd::Zero(12);
  forces << -1.0, 2.0, 1.0, -1.0, 2.0, 1.0, -1.0, -2.0, 1.0, -1.0, -2.0, 1.0;

  // Known solution
  torques_solution << sqrt(2) * ls(1, 1), 0.0, -ls(3, 0), sqrt(2) * ls(1, 1),
      0.0, -ls(3, 0),  // leg 2
      -sqrt(2) * ls(1, 1), 0.0, -ls(3, 0), -sqrt(2) * ls(1, 1), 0.0, -ls(3, 0),
      -4.0, 0.0, 4.0,  // net forces
      0.0, 0.0, 0.0;

  // Compute joint torques
  kinematics.getJacobianGenCoord(state_positions, jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  error = torques - torques_solution;
  toPrint << torques, torques_solution;
  // std::cout << "Test 4:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);
}

TEST(KinematicsTest, testFKIKFeasibleConfigurations) {
  ros::NodeHandle nh;

  // Declare kinematics object
  QuadKD quad;

  // Set up problem variables
  Eigen::Vector3d body_pos = {0, 0, 0};
  Eigen::Vector3d body_rpy = {0, 0, 0};
  Eigen::Vector3d joint_state = {0.7, -0.3 * M_PI, 0 * M_PI};
  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d joint_state_test;
  Eigen::Vector3d foot_pos_world_test;

  // Compute the kinematics
  int N = 10000;
  for (int config = 0; config < N; config++) {
    // Generate valid joint configurations
    Eigen::Vector3d joint_state = {
        (quad.getJointUpperLimit(0) - quad.getJointLowerLimit(0)) *
                (double)rand() / RAND_MAX +
            quad.getJointLowerLimit(0),
        (quad.getJointUpperLimit(1) - quad.getJointLowerLimit(1)) *
                (double)rand() / RAND_MAX +
            quad.getJointLowerLimit(1),
        (quad.getJointUpperLimit(2) - quad.getJointLowerLimit(2)) *
                (double)rand() / RAND_MAX +
            quad.getJointLowerLimit(2)};

    for (int i = 0; i < 4; i++) {
      int leg_index = i;

      // Compute foot positions in this configuration
      quad.worldToFootFKWorldFrame(leg_index, body_pos, body_rpy, joint_state,
                                   foot_pos_world);

      // Run IK to compute corresponding joint angles, then back through FK
      // This ensures that we are enforcing a hip-above-knee configuration if
      // otherwise ambiguous.
      quad.worldToFootIKWorldFrame(leg_index, body_pos, body_rpy,
                                   foot_pos_world, joint_state_test);
      quad.worldToFootFKWorldFrame(leg_index, body_pos, body_rpy,
                                   joint_state_test, foot_pos_world_test);

      // Check the answers
      Eigen::Vector3d error = (foot_pos_world - foot_pos_world_test);
      EXPECT_TRUE(error.norm() <= kinematics_tol);
    }
  }
}

TEST(KinematicsTest, testFKIKInfeasibleConfigurations) {
  ros::NodeHandle nh;

  QuadKD quad;

  // Set up problem variables
  Eigen::Vector3d body_pos = {0, 0, 0};
  Eigen::Vector3d body_rpy = {0, 0, 0};
  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d foot_pos_world_test;
  Eigen::Vector3d joint_state_test;

  // Define arbitrary maximum foot offset for IK testing
  double max_offset = abs(quad.getJointLowerLimit(0)) +
                      quad.getJointLowerLimit(1) + quad.getJointLowerLimit(2);

  // Test random foot positions to make sure nothing breaks
  int N = 10000;
  for (int config = 0; config < N; config++) {
    // Generate random foot offset
    Eigen::Vector3d foot_offset = {
        2 * max_offset * (double)rand() / RAND_MAX - max_offset,
        2 * max_offset * (double)rand() / RAND_MAX - max_offset,
        2 * max_offset * (double)rand() / RAND_MAX - max_offset};

    for (int i = 0; i < 4; i++) {
      int leg_index = i;

      // Transform foot offset into world frame
      Eigen::Vector3d shoulder_pos;
      quad.worldToLegbaseFKWorldFrame(leg_index, body_pos, body_rpy,
                                      shoulder_pos);
      foot_pos_world = shoulder_pos + foot_offset;

      // Run IK and make sure there aren't any errors
      quad.worldToFootIKWorldFrame(leg_index, body_pos, body_rpy,
                                   foot_pos_world, joint_state_test);

      // To do: Check these solutions and make sure they are what we want
    }
  }

  EXPECT_EQ(1 + 1, 2);
}

TEST(KinematicsTest, testBodyToFootFK) {
  ros::NodeHandle nh;

  // Declare kinematics object
  QuadKD quad;

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
  double pitch_min = -0.5 * M_PI;
  double pitch_max = 0.5 * M_PI;
  double yaw_min = -M_PI;
  double yaw_max = M_PI;

  // Compute the kinematics
  int N = 10000;
  for (int config = 0; config < N; config++) {
    // Generate valid joint configurations
    Eigen::Vector3d joint_state = {
        (quad.getJointUpperLimit(0) - quad.getJointLowerLimit(0)) *
                (double)rand() / RAND_MAX +
            quad.getJointLowerLimit(0),
        (quad.getJointUpperLimit(1) - quad.getJointLowerLimit(1)) *
                (double)rand() / RAND_MAX +
            quad.getJointLowerLimit(1),
        (quad.getJointUpperLimit(2) - quad.getJointLowerLimit(2)) *
                (double)rand() / RAND_MAX +
            quad.getJointLowerLimit(2)};

    // Generate valid body orientations
    Eigen::Vector3d body_pos = {
        (pos_max - pos_min) * rand() / RAND_MAX + pos_min,
        (pos_max - pos_min) * rand() / RAND_MAX + pos_min,
        (pos_max - pos_min) * rand() / RAND_MAX + pos_min};

    Eigen::Vector3d body_rpy = {
        (pos_max - roll_min) * rand() / RAND_MAX + roll_min,
        (pitch_max - pitch_min) * rand() / RAND_MAX + pitch_min,
        (yaw_max - yaw_min) * rand() / RAND_MAX + yaw_min};

    Eigen::Matrix4d g_world_body = quad.createAffineMatrix(body_pos, body_rpy);

    for (int leg_index = 0; leg_index < 4; leg_index++) {
      // Compute the foot position in world frame with FK then tranform into
      // body frame
      quad.worldToFootFKWorldFrame(leg_index, body_pos, body_rpy, joint_state,
                                   g_world_foot);
      quad.transformWorldToBody(body_pos, body_rpy, g_world_foot, g_body_foot);
      foot_pos_body = g_body_foot.block<3, 1>(0, 3);

      // Compute foot positions directly from the body frame
      quad.bodyToFootFKBodyFrame(leg_index, joint_state, g_body_foot_test);
      quad.bodyToFootFKBodyFrame(leg_index, joint_state, foot_pos_body_test);

      // Check the answers
      EXPECT_TRUE(foot_pos_body_test.isApprox(foot_pos_body));
      EXPECT_TRUE(g_body_foot_test.isApprox(g_body_foot));
    }
  }
}

TEST(KinematicsTest, testMotorModel) {

  // Declare kinematics object
  QuadKD quad_kd;

  Eigen::VectorXd state_vel(12);
  Eigen::VectorXd valid_input(12);
  Eigen::VectorXd invalid_input(12);
  Eigen::VectorXd constrained_input(12);

  state_vel << 0,0,0,10,10,10,0,0,0,10,10,10;
  valid_input << 10,10,10,10,10,10,-10,-10,-10,-10,-10,-10;
  invalid_input << 40,10,10,10,10,10,-10,-10,-10,-10,-10,-10;

  bool valid_result = quad_kd.applyMotorModel(valid_input, state_vel, constrained_input);
  bool invalid_result = quad_kd.applyMotorModel(invalid_input, state_vel, constrained_input);

  EXPECT_TRUE(valid_result == true);
  EXPECT_TRUE(invalid_result == false);

  int N = 1000;
  int count = 0;
  auto t_start = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    count++;
    bool valid_result = quad_kd.applyMotorModel(valid_input, state_vel, constrained_input);
  }
  auto t_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> t_diff = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
  double average_time = t_diff.count()/count;

  std::cout << "Average applyMotorModel time = " << average_time << " s" << std::endl;

  EXPECT_TRUE(average_time<=1e-6);
}

TEST(KinematicsTest, testConvertCentroidalToFullBody) {
  
  // Declare kinematics object
  QuadKD quad_kd;

  // Declare known variables
  Eigen::VectorXd body_state(12);
  Eigen::VectorXd foot_positions(12);
  Eigen::VectorXd foot_velocities(12);
  Eigen::VectorXd foot_acc(12);
  Eigen::VectorXd grfs(12);
  std::vector<int> contact_mode;

  // Declare unknown variables
  Eigen::VectorXd joint_positions(12);
  Eigen::VectorXd joint_velocities(12);
  Eigen::VectorXd torques(12);
  Eigen::VectorXd state_violation, control_violation;

  // Define terrain map
  grid_map::GridMap map({"z"});
  double map_height = 0;
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.1, grid_map::Position(0.0, 0.0));
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map.getPosition(*it, position);
    map.at("z", *it) = map_height;
  }

  int N_yaw = 10;
  for (int i = 0; i < N_yaw; i++) {

    // Define the nominal standing height and random x,y,yaw
    double h = 0.3;
    double yaw = 2*M_PI*(double)rand()/RAND_MAX - M_PI;
    double x = 2*(double)rand()/RAND_MAX - 1;
    double y = 2*(double)rand()/RAND_MAX - 1;
    grid_map::Position pos = {x,y};
    body_state << x,y,h + map.atPosition("z",pos),0,0,yaw,0,0,0,0,0,0;

    // Extract components of the state
    Eigen::Vector3d body_pos = body_state.segment<3>(0);
    Eigen::Vector3d body_rpy = body_state.segment<3>(3);
    Eigen::VectorXd body_vel = body_state.tail(6);

    // Solve FK for nominal joint angles to get foot positions
    for (int i = 0; i < 4; i++) {
      Eigen::Vector3d nominal_hip_pos_world;
      quad_kd.worldToNominalHipFKWorldFrame(i,body_pos, body_rpy, nominal_hip_pos_world);
      nominal_hip_pos_world[2] = 0;
      foot_positions.segment<3>(3*i) = nominal_hip_pos_world;
    }

    // Define dynamic parameters for a trot
    double m = 11.5;
    double g = 9.81;
    grfs << 0,0,0.5*m*g,0,0,0,0,0,0,0,0,0.5*m*g;
    contact_mode = {1,0,0,1};

    // Define foot velocities (feet not in contact have upwards velocity)
    foot_velocities.setZero();
    double foot_vel_z = 1.0;
    foot_velocities[5] = foot_vel_z;
    foot_velocities[8] = foot_vel_z;
    foot_acc.setZero();

    // Perform conversion
    // quad_kd.convertCentroidalToFullBody(body_state,foot_positions,foot_velocities,
    //   grfs, foot_acc, contact_mode,joint_positions,joint_velocities,torques);
    bool is_exact = quad_kd.convertCentroidalToFullBody(body_state,foot_positions,foot_velocities,grfs,
      joint_positions,joint_velocities,torques);

    // Compute expected joint positions
    double l1 = quad_kd.getLinkLength(0,2);
    Eigen::VectorXd joint_positions_expected(12), joint_velocities_expected(12);
    joint_positions_expected << 0,asin(0.5*h/l1),2*asin(0.5*h/l1),
      0,asin(0.5*h/l1),2*asin(0.5*h/l1),
      0,asin(0.5*h/l1),2*asin(0.5*h/l1),
      0,asin(0.5*h/l1),2*asin(0.5*h/l1);

    // Compute expected joint velocities
    double hip_vel_expected = -0.5*foot_vel_z/(l1*cos(joint_positions_expected[4]));
    double knee_vel_expected = 2*hip_vel_expected;
    joint_velocities_expected << 0,0,0,0,hip_vel_expected,knee_vel_expected,
    0,hip_vel_expected,knee_vel_expected,0,0,0;
    
    // Check joint positions and velocities match
    EXPECT_TRUE(is_exact);
    EXPECT_TRUE(joint_positions.isApprox(joint_positions_expected));
    EXPECT_TRUE(joint_velocities.isApprox(joint_velocities_expected));

    // Check validity
    bool is_state_valid = quad_kd.isValidCentroidalState(body_state,foot_positions,foot_velocities,
      grfs,map, state_violation, control_violation);
    EXPECT_TRUE(is_state_valid);

    body_state[2] += 0.5;
    is_exact = quad_kd.convertCentroidalToFullBody(body_state,foot_positions,foot_velocities,grfs,
      joint_positions,joint_velocities,torques);
    EXPECT_FALSE(is_exact);

    // Check validity
    is_state_valid = quad_kd.isValidCentroidalState(body_state,foot_positions,foot_velocities,grfs,
      map, state_violation, control_violation);
    EXPECT_FALSE(is_state_valid);
  }

  // // Print results if desired
  // std::cout << "joint_positions\n" << joint_positions << std::endl;
  // std::cout << "joint_velocities\n" << joint_velocities << std::endl;
  // std::cout << "torques\n" << torques << std::endl;

  // Check timing characteristics
  int N = 1000;
  int count = 0;
  auto t_start = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    count++;
    quad_kd.convertCentroidalToFullBody(body_state,foot_positions,foot_velocities,grfs,
      joint_positions,joint_velocities,torques);
  }
  auto t_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> t_diff = std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
  double average_time = t_diff.count()/count;

  std::cout << "Average convertCentroidalToFullBody time = " << average_time << " s" << std::endl;

  EXPECT_TRUE(average_time < 1e-4);
}