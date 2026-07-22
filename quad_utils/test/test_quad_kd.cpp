#include "quad_utils/quad_kd2.hpp"
#include "quad_utils/ros_utils.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <array>
#include <cstdlib>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace quad_utils {

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
  std::string xacro_path;
  std::array<std::array<std::string, 3>, 4>
      leg_joint_names;  // [leg][abad,hip,knee]
  std::array<std::array<std::string, 4>, 4>
      leg_frame_names;  // [leg][hip,upper,lower,toe]
  double abad_sign, abad_offset;
  double hip_sign, hip_offset;
  double knee_sign, knee_offset;
};

static RobotKinematicsConfig go2Cfg() {
  RobotKinematicsConfig c;
  c.xacro_path =
      "quad_simulator/go2_description/models/go2/urdf/go2.urdf.xacro";
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
  const char* source_dir = std::getenv("QUAD_UTILS_SOURCE_DIR");
  if (source_dir == nullptr) {
    throw std::runtime_error("Missing QUAD_UTILS_SOURCE_DIR");
  }
  const std::string xacro_path =
      std::string(source_dir) + "/" + cfg.xacro_path;

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

static grid_map::GridMap makeFlatTerrain(double height) {
  grid_map::GridMap map({"z"});
  map.setGeometry(grid_map::Length(10.0, 10.0), 0.1,
                  grid_map::Position(0.0, 0.0));
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    map.at("z", *it) = height;
  }
  return map;
}

const double kinematics_tol = 1e-4;

TEST(KinematicsTest, testDifferentialFKIK) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  RobotKinematicsConfig cfg = go2Cfg();
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
      state.joints.position.push_back(0.0);
      state.joints.position.push_back(0.6);
      state.joints.position.push_back(-1.2);

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

TEST(KinematicsTest, testGo2FootForces) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  RobotKinematicsConfig cfg = go2Cfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  Eigen::VectorXd body_state(12);
  body_state << 0.1, -0.2, 0.35, 0.05, -0.03, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0;
  Eigen::VectorXd joint_positions(12);
  joint_positions << 0.0, 0.6, -1.2, 0.0, 0.7, -1.25, 0.0, 0.55, -1.15, 0.0,
      0.65, -1.2;

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(12, 18);
  kinematics.updateFromBodyJoints(body_state, joint_positions);
  kinematics.getJacobianGenCoord(jacobian);

  Eigen::VectorXd forces(12);
  forces << 2.0, 0.0, 5.0, -1.0, 0.5, 4.0, 1.5, -0.2, 3.0, 0.0, 1.0, 6.0;
  Eigen::VectorXd generalized_velocity(18);
  generalized_velocity << 0.1, -0.2, 0.3, -0.1, 0.2, -0.3, 0.15, -0.25, 0.35,
      -0.15, 0.25, -0.35, 0.4, -0.2, 0.1, 0.05, -0.03, 0.02;

  const Eigen::VectorXd torques = jacobian.transpose() * forces;
  const double foot_power = forces.dot(jacobian * generalized_velocity);
  const double generalized_power = torques.dot(generalized_velocity);

  EXPECT_EQ(jacobian.rows(), 12);
  EXPECT_EQ(jacobian.cols(), 18);
  EXPECT_NEAR(foot_power, generalized_power, kinematics_tol);
}

TEST(KinematicsTest, testGo2PinocchioStateAssembly) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_assembly_test");

  RobotKinematicsConfig cfg = go2Cfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  Eigen::VectorXd body_state(12);
  body_state << 1.0, -2.0, 0.4, 0.1, -0.2, 0.3, 0.5, -0.25, 0.75, 0.2,
      -0.1, 0.05;
  Eigen::VectorXd joint_positions(12);
  joint_positions << 0.01, 0.02, 0.03, 0.11, 0.12, 0.13, 0.21, 0.22, 0.23,
      0.31, 0.32, 0.33;
  Eigen::VectorXd joint_velocities(12);
  joint_velocities << 1.0, 2.0, 3.0, 1.1, 1.2, 1.3, 2.1, 2.2, 2.3, 3.1,
      3.2, 3.3;

  Eigen::VectorXd q, v;
  kinematics.assembleQVFromBodyAndJoints(body_state, joint_positions,
                                         joint_velocities, q, v);

  ASSERT_EQ(q.size(), kinematics.model().nq);
  ASSERT_EQ(v.size(), kinematics.model().nv);
  EXPECT_TRUE(q.segment<3>(0).isApprox(body_state.segment<3>(0)));

  Eigen::Quaterniond expected_quat =
      Eigen::AngleAxisd(body_state(5), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(body_state(4), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(body_state(3), Eigen::Vector3d::UnitX());
  Eigen::Vector4d expected_qxyzw(expected_quat.x(), expected_quat.y(),
                                 expected_quat.z(), expected_quat.w());
  EXPECT_TRUE(q.segment<4>(3).isApprox(expected_qxyzw, kinematics_tol));

  const Eigen::Vector3d expected_base_linear =
      expected_quat.toRotationMatrix().transpose() * body_state.segment<3>(6);
  EXPECT_TRUE(v.segment<3>(0).isApprox(expected_base_linear, kinematics_tol));
  EXPECT_TRUE(v.segment<3>(3).isApprox(body_state.segment<3>(9)));

  const auto ordered_names = kinematics.getOrderedJointNames();
  ASSERT_EQ(ordered_names.size(), 12u);
  for (int i = 0; i < 12; ++i) {
    const auto joint_id = kinematics.model().getJointId(ordered_names[i]);
    ASSERT_LT(joint_id, kinematics.model().joints.size());
    EXPECT_NEAR(q[kinematics.model().idx_qs[joint_id]],
                joint_positions[i], kinematics_tol);
    EXPECT_NEAR(v[kinematics.model().idx_vs[joint_id]],
                joint_velocities[i], kinematics_tol);
  }
}

TEST(KinematicsTest, testFKIKFeasibleConfigurations) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  RobotKinematicsConfig cfg = go2Cfg();
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
      kinematics.worldToFootIKWorldFrame(leg, body_pos, body_rpy,
                                         foot_pos_world, joint_state_test);

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

  RobotKinematicsConfig cfg = go2Cfg();
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
    kinematics.applyMotorModel(valid_input, state_vel, constrained_input);
  }
  auto t_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> t_diff =
      std::chrono::duration_cast<std::chrono::duration<double>>(t_end -
                                                                t_start);
  double average_time = t_diff.count() / count;

  std::cout << "Average applyMotorModel time = " << average_time << " s"
            << std::endl;
}

TEST(KinematicsTest, testBodyToFootFK) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_compare_test");

  RobotKinematicsConfig cfg = go2Cfg();
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

  RobotKinematicsConfig cfg = go2Cfg();
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
}

TEST(KinematicsTest, testGo2InverseDynamicsReturnsFiniteTorques) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_inverse_dynamics_test");

  RobotKinematicsConfig cfg = go2Cfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  Eigen::VectorXd body_state(12);
  body_state << 0.0, 0.0, 0.35, 0.0, 0.0, 0.0, 0.1, -0.1, 0.0, 0.0, 0.0,
      0.1;
  Eigen::VectorXd joint_positions(12);
  joint_positions << 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0,
      0.6, -1.2;
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(12);
  kinematics.updateFromBodyJoints(body_state, joint_positions,
                                  joint_velocities);

  Eigen::VectorXd foot_acc = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd grf = Eigen::VectorXd::Zero(12);
  grf << 0.0, 0.0, 55.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 55.0;
  std::vector<int> contact_mode{1, 0, 0, 1};
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

  kinematics.computeInverseDynamics(foot_acc, grf, contact_mode, tau);

  EXPECT_EQ(tau.size(), 12);
  EXPECT_TRUE(tau.allFinite());
}

TEST(KinematicsTest, testGo2FullStateValidityReportsStateAndControlFailures) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node = std::make_shared<rclcpp::Node>("kinematics_validity_test");

  RobotKinematicsConfig cfg = go2Cfg();
  loadRobotParams(node, cfg);

  quad_utils::QuadKD2 kinematics(node);

  Eigen::VectorXd body_state(12);
  body_state << 0.0, 0.0, 0.35, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0;
  Eigen::VectorXd joint_positions(12);
  joint_positions << 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0, 0.6, -1.2, 0.0,
      0.6, -1.2;
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd joint_torques = Eigen::VectorXd::Zero(12);
  Eigen::VectorXd state_violation, control_violation;

  kinematics.updateFromBodyJoints(body_state, joint_positions,
                                  joint_velocities);
  EXPECT_TRUE(kinematics.isValidFullState(
      body_state, joint_positions, joint_velocities, joint_torques,
      makeFlatTerrain(0.0), state_violation, control_violation));
  EXPECT_TRUE((state_violation.array() >= 0.0).all());
  EXPECT_TRUE(control_violation.isZero());

  EXPECT_FALSE(kinematics.isValidFullState(
      body_state, joint_positions, joint_velocities, joint_torques,
      makeFlatTerrain(10.0), state_violation, control_violation));
  EXPECT_TRUE((state_violation.array() < 0.0).any());
  EXPECT_TRUE(control_violation.isZero());

  Eigen::VectorXd excessive_torques = Eigen::VectorXd::Zero(12);
  excessive_torques[0] = 100.0;
  EXPECT_FALSE(kinematics.isValidFullState(
      body_state, joint_positions, joint_velocities, excessive_torques,
      makeFlatTerrain(0.0), state_violation, control_violation));
  EXPECT_TRUE((state_violation.array() >= 0.0).all());
  EXPECT_LT(control_violation[0], 0.0);
}

}  // namespace quad_utils
