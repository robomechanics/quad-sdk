#include "quad_utils/quad_kd2.hpp"
#include "quad_utils/ros_utils.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace quad_utils {

static grid_map::GridMap makeFlatMap(double height) {
  grid_map::GridMap map({"z"});
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.1,
                  grid_map::Position(0.0, 0.0));
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("z", *it) = height;
  }
  return map;
}

static std::string runXacro(const std::string& xacro_path) {
  std::string cmd = "xacro " + xacro_path;
  std::array<char, 4096> buffer;
  std::string result;

  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    throw std::runtime_error("Failed to run xacro");
  }
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  pclose(pipe);
  return result;
}

struct RobotKinematicsConfig {
  std::string xacro_pkg;
  std::string xacro_relpath;  // relative inside pkg share
  std::array<std::array<std::string, 3>, 4>
      leg_joint_names;  // [leg][abad,hip,knee]
  std::array<std::array<std::string, 4>, 4>
      leg_frame_names;  // [leg][hip,upper,lower,toe]
  double abad_sign, abad_offset;
  double hip_sign, hip_offset;
  double knee_sign, knee_offset;
};

static RobotKinematicsConfig spiritCfg() {
  RobotKinematicsConfig c;
  c.xacro_pkg = "spirit_description";
  c.xacro_relpath = "models/spirit/urdf/spirit.urdf.xacro";
  c.leg_joint_names = {{{{"8", "0", "1"}},
                        {{"9", "2", "3"}},
                        {{"10", "4", "5"}},
                        {{"11", "6", "7"}}}};
  c.leg_frame_names = {{{{"hip0", "upper0", "lower0", "toe0"}},
                        {{"hip1", "upper1", "lower1", "toe1"}},
                        {{"hip2", "upper2", "lower2", "toe2"}},
                        {{"hip3", "upper3", "lower3", "toe3"}}}};
  c.abad_sign = 1.0;
  c.abad_offset = 0.0;
  c.hip_sign = 1.0;
  c.hip_offset = 0.0;
  c.knee_sign = 1.0;
  c.knee_offset = 0.0;
  // c.knee_sign = 1.0; c.knee_offset = M_PI;
  return c;
}

static RobotKinematicsConfig go2Cfg() {
  RobotKinematicsConfig c;
  c.xacro_pkg = "go2_description";
  c.xacro_relpath = "models/go2/urdf/go2.urdf.xacro";
  c.leg_joint_names = {{{{"8", "0", "1"}},
                        {{"9", "2", "3"}},
                        {{"10", "4", "5"}},
                        {{"11", "6", "7"}}}};
  c.leg_frame_names = {{{{"hip0", "upper0", "lower0", "toe0"}},
                        {{"hip1", "upper1", "lower1", "toe1"}},
                        {{"hip2", "upper2", "lower2", "toe2"}},
                        {{"hip3", "upper3", "lower3", "toe3"}}}};
  c.abad_sign = 1.0;
  c.abad_offset = 0.0;
  c.hip_sign = -1.0;
  c.hip_offset = M_PI / 2.0;
  c.knee_sign = 1.0;
  c.knee_offset = -M_PI;
  return c;
}

static void loadRobotParams(const rclcpp::Node::SharedPtr& node,
                            const RobotKinematicsConfig& cfg) {
  const std::string xacro_path =
      ament_index_cpp::get_package_share_directory(cfg.xacro_pkg) + "/" +
      cfg.xacro_relpath;

  const std::string urdf_string = runXacro(xacro_path);
  node->declare_parameter<std::string>("robot_description", urdf_string);

  for (int i = 0; i < 4; i++) {
    const std::string p = "leg_" + std::to_string(i);

    node->declare_parameter(p + ".joints.abad.name", cfg.leg_joint_names[i][0]);
    node->declare_parameter(p + ".joints.hip.name", cfg.leg_joint_names[i][1]);
    node->declare_parameter(p + ".joints.knee.name", cfg.leg_joint_names[i][2]);

    node->declare_parameter(p + ".joints.abad.sign", cfg.abad_sign);
    node->declare_parameter(p + ".joints.abad.offset", cfg.abad_offset);
    node->declare_parameter(p + ".joints.hip.sign", cfg.hip_sign);
    node->declare_parameter(p + ".joints.hip.offset", cfg.hip_offset);
    node->declare_parameter(p + ".joints.knee.sign", cfg.knee_sign);
    node->declare_parameter(p + ".joints.knee.offset", cfg.knee_offset);

    node->declare_parameter(p + ".frames.hip", cfg.leg_frame_names[i][0]);
    node->declare_parameter(p + ".frames.upper", cfg.leg_frame_names[i][1]);
    node->declare_parameter(p + ".frames.lower", cfg.leg_frame_names[i][2]);
    node->declare_parameter(p + ".frames.toe", cfg.leg_frame_names[i][3]);
  }
}

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testDifferentialFKIK) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  // Load in Robot Specifc Params
  const std::string robot = "go2";
  RobotKinematicsConfig cfg = (robot == "go2") ? go2Cfg() : spiritCfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);
  const int num_tests = 20;

  for (size_t i = 0; i < num_tests; i++) {
    // Declare input and output Robot State object
    quad_msgs::msg::RobotState state, state_out;

    // Random Velocities at Origin
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
      if (robot == "go2") {
        state.joints.position.push_back(0.0);
        state.joints.position.push_back(0.6);
        state.joints.position.push_back(-1.2);
      } else if (robot == "spirit") {  // spirit
        state.joints.position.push_back(0.1);
        state.joints.position.push_back(0.2);
        state.joints.position.push_back(0.3);
      } else {
        state.joints.position.push_back(0.1);
        state.joints.position.push_back(0.2);
        state.joints.position.push_back(0.3);
      }

      // Random joints velocity
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);
      state.joints.velocity.push_back(3.14 * (double)rand() / RAND_MAX - 1.57);

      // We don't need joints effort here
      state.joints.effort.push_back(0.0);
      state.joints.effort.push_back(0.0);
      state.joints.effort.push_back(0.0);
    }

    // Run FK to get foot velocities and IK them back
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

TEST(KinematicsTest, testSpiritFootForces) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  // Load in Robot Specifc Params
  const std::string robot = "spirit";
  RobotKinematicsConfig cfg = (robot == "go2") ? go2Cfg() : spiritCfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  // Length parameters from URDF - Grab from kinematics object
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
  Eigen::VectorXd body_state(12);  // your helper expects size >= 6; 12 is fine
  body_state.setZero();
  body_state.segment<3>(0) = state_positions.segment<3>(12);  // x,y,z
  body_state.segment<3>(3) = state_positions.segment<3>(15);  // r,p,y

  Eigen::VectorXd joint_positions(12);
  joint_positions = state_positions.head<12>();

  kinematics.updateFromBodyJoints(body_state, joint_positions);
  kinematics.getJacobianGenCoord(jacobian);
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
  body_state.setZero();
  body_state.segment<3>(0) = state_positions.segment<3>(12);  // x,y,z
  body_state.segment<3>(3) = state_positions.segment<3>(15);  // r,p,y

  joint_positions.setZero();
  joint_positions = state_positions.head<12>();

  kinematics.updateFromBodyJoints(body_state, joint_positions);
  kinematics.getJacobianGenCoord(jacobian);
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
  body_state.setZero();
  body_state.segment<3>(0) = state_positions.segment<3>(12);  // x,y,z
  body_state.segment<3>(3) = state_positions.segment<3>(15);  // r,p,y

  joint_positions.setZero();
  joint_positions = state_positions.head<12>();

  kinematics.updateFromBodyJoints(body_state, joint_positions);
  kinematics.getJacobianGenCoord(jacobian);
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
  body_state.setZero();
  body_state.segment<3>(0) = state_positions.segment<3>(12);  // x,y,z
  body_state.segment<3>(3) = state_positions.segment<3>(15);  // r,p,y

  joint_positions.setZero();
  joint_positions = state_positions.head<12>();

  kinematics.updateFromBodyJoints(body_state, joint_positions);
  kinematics.getJacobianGenCoord(jacobian);
  torques = jacobian.transpose() * forces;

  // Check the answers
  error = torques - torques_solution;
  toPrint << torques, torques_solution;
  // std::cout << "Test 4:\n" << toPrint << std::endl;
  EXPECT_TRUE(error.norm() <= kinematics_tol);
}

TEST(KinematicsTest, testFKIKFeasibleConfigurations) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  // Load in Robot Specifc Params
  const std::string robot = "go2";
  RobotKinematicsConfig cfg = (robot == "go2") ? go2Cfg() : spiritCfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  // Base state (pos + rpy) keep fixed for this test
  Eigen::VectorXd body_state(12);
  body_state.setZero();
  Eigen::Vector3d body_pos = body_state.segment<3>(0);
  Eigen::Vector3d body_rpy = body_state.segment<3>(3);

  Eigen::VectorXd joint_positions(12);
  joint_positions.setZero();

  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d joint_state_test;
  Eigen::Vector3d foot_pos_world_test;

  // Compute the Kinematics
  int N = 10000;
  for (int config = 0; config < N; config++) {
    // Sample a full joint configuration for all 4 legs
    for (int leg_index = 0; leg_index < 4; ++leg_index) {
      joint_positions.segment<3>(3 * leg_index)
          << (kinematics.getJointUpperLimit(leg_index, 0) -
              kinematics.getJointLowerLimit(leg_index, 0)) *
                     (double)rand() / RAND_MAX +
                 kinematics.getJointLowerLimit(leg_index, 0),
          (kinematics.getJointUpperLimit(leg_index, 1) -
           kinematics.getJointLowerLimit(leg_index, 1)) *
                  (double)rand() / RAND_MAX +
              kinematics.getJointLowerLimit(leg_index, 1),
          (kinematics.getJointUpperLimit(leg_index, 2) -
           kinematics.getJointLowerLimit(leg_index, 2)) *
                  (double)rand() / RAND_MAX +
              kinematics.getJointLowerLimit(leg_index, 2);
    }

    // Update the model once for this configuration
    kinematics.updateFromBodyJoints(body_state, joint_positions);

    // Per Leg FK-> IK -> FK
    for (int leg = 0; leg < 4; ++leg) {
      // Compute foot positions in this configuration
      kinematics.worldToFootFKWorldFrame(leg, foot_pos_world);

      // Run IK to compute corresponding joint angles, then back through FK
      // This ensures that we are enforcing a hip-above-knee configuration if
      // otherwise ambiguous.
      bool exact = kinematics.worldToFootIKWorldFrame(
          leg, body_pos, body_rpy, foot_pos_world, joint_state_test);

      const Eigen::Vector3d joint_state = joint_positions.segment<3>(3 * leg);
      // Skip if original configuration was in an alternate configuration
      if (!joint_state_test.isApprox(joint_state)) continue;

      // Build a FULL joint vector with this leg replaced
      Eigen::VectorXd joint_positions_test = joint_positions;
      joint_positions_test.segment<3>(3 * leg) = joint_state_test;

      // Update and FK again
      kinematics.updateFromBodyJoints(body_state, joint_positions_test);
      kinematics.worldToFootFKWorldFrame(leg, foot_pos_world_test);

      // Check the answers
      Eigen::Vector3d error = (foot_pos_world - foot_pos_world_test);
      EXPECT_LE(error.norm(), kinematics_tol);
    }
  }
}

TEST(KinematicsTest, testMotorModel) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  // Load in Robot Specifc Params
  const std::string robot = "go2";
  RobotKinematicsConfig cfg = (robot == "go2") ? go2Cfg() : spiritCfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  Eigen::VectorXd state_vel(12);
  Eigen::VectorXd valid_input(12);
  Eigen::VectorXd invalid_input(12);
  Eigen::VectorXd constrained_input(12);

  state_vel << 0, 0, 0, 10, 10, 10, 0, 0, 0, 10, 10, 10;
  valid_input << 10, 10, 10, 10, 10, 10, -10, -10, -10, -10, -10, -10;
  invalid_input << 40, 10, 10, 10, 10, 10, -10, -10, -10, -10, -10, -10;

  bool valid_result =
      kinematics.applyMotorModel(valid_input, state_vel, constrained_input);
  bool invalid_result =
      kinematics.applyMotorModel(invalid_input, state_vel, constrained_input);

  EXPECT_TRUE(valid_result == true);
  EXPECT_TRUE(invalid_result == false);

  int N = 1000;
  int count = 0;
  auto t_start = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    count++;
    bool valid_result =
        kinematics.applyMotorModel(valid_input, state_vel, constrained_input);
  }
  auto t_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> t_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                t_start);
  double average_time = t_diff.count() / count;

  std::cout << "Average applyMotorModel time = " << average_time << " s"
            << std::endl;

  EXPECT_TRUE(average_time <= 1e-6);
}

TEST(KinematicsTest, testBodyToFootFK) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  // Load in Robot Specifc Params
  const std::string robot = "go2";
  RobotKinematicsConfig cfg = (robot == "go2") ? go2Cfg() : spiritCfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  // Set up problem variables
  Eigen::Matrix4d g_world_foot;
  Eigen::Matrix4d g_body_foot;
  Eigen::Vector3d foot_pos_body;

  Eigen::Matrix4d g_body_foot_test;
  Eigen::Vector3d foot_pos_body_test;

  Eigen::VectorXd body_state(12);
  Eigen::VectorXd joint_positions(12);
  body_state.setZero();
  joint_positions.setZero();

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
    // Generate valid body orientations
    Eigen::Vector3d body_pos = {
        (pos_max - pos_min) * rand() / RAND_MAX + pos_min,
        (pos_max - pos_min) * rand() / RAND_MAX + pos_min,
        (pos_max - pos_min) * rand() / RAND_MAX + pos_min};

    Eigen::Vector3d body_rpy = {
        (roll_max - roll_min) * rand() / RAND_MAX + roll_min,
        (pitch_max - pitch_min) * rand() / RAND_MAX + pitch_min,
        (yaw_max - yaw_min) * rand() / RAND_MAX + yaw_min};

    Eigen::Matrix4d g_world_body =
        kinematics.createAffineMatrix(body_pos, body_rpy);

    // Compose a Complete State for Update
    body_state.segment<3>(0) = body_pos;
    body_state.segment<3>(3) = body_rpy;
    for (int leg_index = 0; leg_index < 4; leg_index++) {
      joint_positions.segment<3>(3 * leg_index)
          << (kinematics.getJointUpperLimit(leg_index, 0) -
              kinematics.getJointLowerLimit(leg_index, 0)) *
                     (double)rand() / RAND_MAX +
                 kinematics.getJointLowerLimit(leg_index, 0),
          (kinematics.getJointUpperLimit(leg_index, 1) -
           kinematics.getJointLowerLimit(leg_index, 1)) *
                  (double)rand() / RAND_MAX +
              kinematics.getJointLowerLimit(leg_index, 1),
          (kinematics.getJointUpperLimit(leg_index, 2) -
           kinematics.getJointLowerLimit(leg_index, 2)) *
                  (double)rand() / RAND_MAX +
              kinematics.getJointLowerLimit(leg_index, 2);
    }
    kinematics.updateFromBodyJoints(body_state, joint_positions);
    for (int leg_index = 0; leg_index < 4; leg_index++) {
      // Compute the foot position in world frame with FK then tranform into
      // body frame
      kinematics.worldToFootFKWorldFrame(leg_index, g_world_foot);
      kinematics.transformWorldToBody(body_pos, body_rpy, g_world_foot,
                                      g_body_foot);
      foot_pos_body = g_body_foot.block<3, 1>(0, 3);

      // Compute foot positions directly from the body frame
      kinematics.bodyToFootFKBodyFrame(leg_index, g_body_foot_test);
      kinematics.bodyToFootFKBodyFrame(leg_index, foot_pos_body_test);

      // Check the answers
      EXPECT_TRUE(foot_pos_body_test.isApprox(foot_pos_body));
      EXPECT_TRUE(g_body_foot_test.isApprox(g_body_foot));
    }
  }
}

TEST(KinematicsTest, testConvertCentroidalToFullBody) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  // Load in Robot Specifc Params
  const std::string robot = "spirit";
  RobotKinematicsConfig cfg = (robot == "go2") ? go2Cfg() : spiritCfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

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
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.1,
                  grid_map::Position(0.0, 0.0));
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map.getPosition(*it, position);
    map.at("z", *it) = map_height;
  }

  int N_yaw = 10;
  for (int i = 0; i < N_yaw; i++) {
    // Define the nominal standing height and random x,y,yaw
    double h = 0.3;
    double yaw = 2 * M_PI * (double)rand() / RAND_MAX - M_PI;
    double x = 2 * (double)rand() / RAND_MAX - 1;
    double y = 2 * (double)rand() / RAND_MAX - 1;
    grid_map::Position pos = {x, y};
    body_state << x, y, h + map.atPosition("z", pos), 0, 0, yaw, 0, 0, 0, 0, 0,
        0;

    // Extract components of the state
    Eigen::Vector3d body_pos = body_state.segment<3>(0);
    Eigen::Vector3d body_rpy = body_state.segment<3>(3);
    Eigen::VectorXd body_vel = body_state.tail(6);

    // Solve FK for nominal joint angles to get foot positions
    for (int i = 0; i < 4; i++) {
      Eigen::Vector3d nominal_hip_pos_world;
      kinematics.worldToNominalHipFKWorldFrame(i, body_pos, body_rpy,
                                               nominal_hip_pos_world);
      nominal_hip_pos_world[2] = 0;
      foot_positions.segment<3>(3 * i) = nominal_hip_pos_world;
    }

    // Define dynamic parameters for a trot
    double m = 11.5;
    double g = 9.81;
    grfs << 0, 0, 0.5 * m * g, 0, 0, 0, 0, 0, 0, 0, 0, 0.5 * m * g;
    contact_mode = {1, 0, 0, 1};

    // Define foot velocities (feet not in contact have upwards velocity)
    foot_velocities.setZero();
    double foot_vel_z = 1.0;
    foot_velocities[5] = foot_vel_z;
    foot_velocities[8] = foot_vel_z;
    foot_acc.setZero();

    // Perform conversion
    bool is_exact = kinematics.convertCentroidalToFullBody(
        body_state, foot_positions, foot_velocities, grfs, joint_positions,
        joint_velocities, torques);

    // Compute expected joint positions (IK angles transformed to Pinocchio
    // space)
    double l1 = kinematics.getLinkLength(0, 2);
    double hip_ik = asin(0.5 * h / l1);
    double knee_ik = 2 * hip_ik;
    double hip_pin = hip_ik * cfg.hip_sign + cfg.hip_offset;
    double knee_pin = knee_ik * cfg.knee_sign + cfg.knee_offset;
    Eigen::VectorXd joint_positions_expected(12), joint_velocities_expected(12);
    joint_positions_expected << 0, hip_pin, knee_pin, 0, hip_pin, knee_pin, 0,
        hip_pin, knee_pin, 0, hip_pin, knee_pin;

    // Compute expected joint velocities (IK velocities transformed to Pinocchio
    // space)
    double hip_vel_ik = -0.5 * foot_vel_z / (l1 * cos(hip_ik));
    double knee_vel_ik = 2 * hip_vel_ik;
    double hip_vel_pin = hip_vel_ik * cfg.hip_sign;
    double knee_vel_pin = knee_vel_ik * cfg.knee_sign;
    joint_velocities_expected << 0, 0, 0, 0, hip_vel_pin, knee_vel_pin, 0,
        hip_vel_pin, knee_vel_pin, 0, 0, 0;

    // Check joint positions and velocities match
    EXPECT_TRUE(is_exact);
    EXPECT_TRUE(joint_positions.isApprox(joint_positions_expected));
    EXPECT_TRUE(joint_velocities.isApprox(joint_velocities_expected));

    // Check validity
    bool is_state_valid = kinematics.isValidCentroidalState(
        body_state, foot_positions, foot_velocities, grfs, map, joint_positions,
        joint_velocities, torques, state_violation, control_violation);
    EXPECT_TRUE(is_state_valid);

    body_state[2] += 0.5;
    is_exact = kinematics.convertCentroidalToFullBody(
        body_state, foot_positions, foot_velocities, grfs, joint_positions,
        joint_velocities, torques);
    EXPECT_FALSE(is_exact);

    // Check validity
    is_state_valid = kinematics.isValidCentroidalState(
        body_state, foot_positions, foot_velocities, grfs, map, joint_positions,
        joint_velocities, torques, state_violation, control_violation);
    EXPECT_FALSE(is_state_valid);
  }

  // Check timing characteristics
  int N = 1000;
  int count = 0;
  auto t_start = std::chrono::steady_clock::now();
  for (int i = 0; i < N; i++) {
    count++;
    kinematics.convertCentroidalToFullBody(
        body_state, foot_positions, foot_velocities, grfs, joint_positions,
        joint_velocities, torques);
  }
  auto t_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> t_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                t_start);
  double average_time = t_diff.count() / count;

  std::cout << "Average convertCentroidalToFullBody time = " << average_time
            << " s" << std::endl;

  EXPECT_TRUE(average_time < 1e-4);
}

}  // namespace quad_utils
