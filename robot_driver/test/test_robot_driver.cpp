#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "quad_utils/quad_kd2.hpp"
#include "robot_driver/controllers/grf_pid_controller.hpp"
#include "robot_driver/controllers/inertia_estimation_controller.hpp"
#include "robot_driver/controllers/inverse_dynamics_controller.hpp"
#include "robot_driver/controllers/joint_controller.hpp"
#include "robot_driver/controllers/leg_controller.hpp"
#include "robot_driver/controllers/underbrush_inverse_dynamics.hpp"
#include "robot_driver/estimators/comp_filter_estimator.hpp"
#include "robot_driver/estimators/ekf_estimator.hpp"
#include "robot_driver/estimators/state_estimator.hpp"
#include "robot_driver/hardware_interfaces/unitree_interface.hpp"
#define private public
#define protected public
#include "robot_driver/robot_driver.hpp"
#undef protected
#undef private
#include "robot_driver/robot_driver_utils.hpp"

int my_argc;
char** my_argv;

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

static rclcpp::NodeOptions makeRobotDriverOptions(
    const std::string& controller = "inverse_dynamics",
    const std::string& estimator = "none") {
  const std::string rd_pkg_share =
      ament_index_cpp::get_package_share_directory("robot_driver");
  const std::string utils_pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  const std::string robot_driver_param_file =
      rd_pkg_share + "/config/robot_driver.yaml";
  const std::string robot_driver_topics_file =
      rd_pkg_share + "/config/robot_driver_topics.yaml";
  const std::string robot_specific_param_file =
      utils_pkg_share + "/config/go2.yaml";
  const char* source_dir = std::getenv("ROBOT_DRIVER_SOURCE_DIR");
  if (source_dir == nullptr) {
    throw std::runtime_error("Missing ROBOT_DRIVER_SOURCE_DIR");
  }
  const std::string xacro_path =
      std::string(source_dir) +
      "/quad_simulator/go2_description/models/go2/urdf/go2.urdf.xacro";
  const std::string urdf_string = runXacro(xacro_path);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", robot_driver_param_file,
                     "--params-file", robot_driver_topics_file, "--params-file",
                     robot_specific_param_file});
  options.append_parameter_override("use_sim_time", false);
  options.append_parameter_override("is_hardware", false);
  options.append_parameter_override("robot_type", "go2");
  options.append_parameter_override("controller", controller);
  options.append_parameter_override("estimator_id", estimator);
  options.append_parameter_override("debug_estimator_id", "none");
  options.append_parameter_override("robot_description", urdf_string);
  return options;
}

static rclcpp::Node::SharedPtr makeNode(
    const std::string& /*name*/ = "robot_driver",
    const std::string& controller = "inverse_dynamics",
    const std::string& estimator = "none") {
  return std::make_shared<rclcpp::Node>(
      "robot_driver", "robot_1",
      makeRobotDriverOptions(controller, estimator));
}

class TestLegController : public LegController {
 public:
  TestLegController(rclcpp::Node::SharedPtr node,
                    std::shared_ptr<quad_utils::QuadKD2> quadKD = nullptr)
      : LegController(node, "robot_1", quadKD) {}

  bool computeLegCommandArray(
      const quad_msgs::msg::RobotState&,
      quad_msgs::msg::LegCommandArray&,
      quad_msgs::msg::GRFArray&) override {
    return false;
  }

  const std::vector<double>& stanceKp() const { return stance_kp_; }
  const std::vector<double>& stanceKd() const { return stance_kd_; }
  const std::vector<double>& swingKp() const { return swing_kp_; }
  const std::vector<double>& swingKd() const { return swing_kd_; }
  const std::vector<double>& swingKpCart() const { return swing_kp_cart_; }
  const std::vector<double>& swingKdCart() const { return swing_kd_cart_; }
  const std::string& modelPath() const { return model_path_; }
  quad_msgs::msg::RobotPlan::SharedPtr localPlan() const {
    return last_local_plan_msg_;
  }
  rclcpp::Time localPlanTime() const { return last_local_plan_time_; }
};

class TestStateEstimator : public StateEstimator {
 public:
  explicit TestStateEstimator(rclcpp::Node::SharedPtr node)
      : StateEstimator(node, "robot_1", nullptr) {}

  void init() override {}

  bool updateOnce(quad_msgs::msg::RobotState&) override { return false; }

  bool footContactReceived() const { return foot_contact_received_; }
  const quad_msgs::msg::FootContact& lastFootContact() const {
    return last_foot_contact_msg_;
  }
  geometry_msgs::msg::PoseStamped::SharedPtr lastMocap() const {
    return last_mocap_msg_;
  }
};

class TestUnitreeInterface : public UnitreeInterface {
 public:
  explicit TestUnitreeInterface(const std::string& robot_name = "go2")
      : UnitreeInterface(robot_name) {}

  using UnitreeInterface::crc32Core;
  using UnitreeInterface::getFootForcesRaw;
  using UnitreeInterface::lowStateHandler;

  const std::string& robotName() const { return robot_name_; }
  int numMotors() const { return num_motors_; }
  bool hasWheels() const { return has_wheels_; }
};

static std::shared_ptr<quad_utils::QuadKD2> makeGo2Kinematics(
    const rclcpp::Node::SharedPtr& node) {
  return std::make_shared<quad_utils::QuadKD2>(node);
}

static quad_msgs::msg::GRFArray makeGrfArray(double stamp_s = 0.0) {
  quad_msgs::msg::GRFArray grfs;
  grfs.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_s * 1e9),
                                   RCL_ROS_TIME);
  grfs.header.frame_id = "map";
  grfs.points.resize(4);
  grfs.vectors.resize(4);
  grfs.contact_states = {true, true, true, true};
  for (int i = 0; i < 4; ++i) {
    grfs.points[i].z = 0.0;
    grfs.vectors[i].z = 30.0;
  }
  return grfs;
}

static quad_msgs::msg::RobotState makeGo2RobotState(
    quad_utils::QuadKD2& kinematics, double stamp_s = 0.0) {
  quad_msgs::msg::RobotState state;
  state.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_s * 1e9),
                                    RCL_ROS_TIME);
  state.header.frame_id = "map";
  state.body.pose.position.z = 0.35;
  state.body.pose.orientation.w = 1.0;
  state.joints.name = kinematics.getOrderedJointNames();
  state.joints.position = {0.0, 0.6, -1.2, 0.0, 0.6, -1.2,
                           0.0, 0.6, -1.2, 0.0, 0.6, -1.2};
  state.joints.velocity.resize(12, 0.0);
  state.joints.effort.resize(12, 0.0);
  state.body.twist.linear.x = 0.0;
  state.body.twist.linear.y = 0.0;
  state.body.twist.linear.z = 0.0;
  state.body.twist.angular.x = 0.0;
  state.body.twist.angular.y = 0.0;
  state.body.twist.angular.z = 0.0;

  quad_utils::fkRobotState(kinematics, state);
  for (auto& foot : state.feet.feet) {
    foot.contact = true;
  }
  return state;
}

static quad_msgs::msg::RobotPlan::SharedPtr makeFreshGo2Plan(
    const rclcpp::Node::SharedPtr& node, quad_utils::QuadKD2& kinematics) {
  auto plan = std::make_shared<quad_msgs::msg::RobotPlan>();
  plan->header.stamp = node->now();
  plan->header.frame_id = "map";
  plan->state_timestamp = node->now();
  plan->states = {makeGo2RobotState(kinematics, 0.0),
                  makeGo2RobotState(kinematics, 0.2)};
  plan->grfs = {makeGrfArray(0.0), makeGrfArray(0.2)};
  plan->primitive_ids = {0, 0};
  return plan;
}

static void expectFourLegCommands(
    const quad_msgs::msg::LegCommandArray& leg_commands) {
  ASSERT_EQ(leg_commands.leg_commands.size(), 4u);
  for (const auto& leg : leg_commands.leg_commands) {
    ASSERT_EQ(leg.motor_commands.size(), 3u);
    for (const auto& motor : leg.motor_commands) {
      EXPECT_TRUE(std::isfinite(motor.pos_setpoint));
      EXPECT_TRUE(std::isfinite(motor.vel_setpoint));
      EXPECT_TRUE(std::isfinite(motor.torque_ff));
      EXPECT_TRUE(std::isfinite(motor.kp));
      EXPECT_TRUE(std::isfinite(motor.kd));
    }
  }
}

template <typename MsgT>
static bool spinUntilMessage(
    const rclcpp::Node::SharedPtr& node_a,
    const rclcpp::Node::SharedPtr& node_b,
    const std::shared_ptr<MsgT>& msg,
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(750)) {
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !msg &&
         std::chrono::steady_clock::now() - start < timeout) {
    rclcpp::spin_some(node_a);
    rclcpp::spin_some(node_b);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return msg != nullptr;
}

TEST(RobotDriverUtils, LoadMotorCommandMsgCopiesAllFields) {
  quad_msgs::msg::MotorCommand msg;
  robot_driver_utils::loadMotorCommandMsg(1.2, -0.5, 3.4, 40.0, 2.5, msg);

  EXPECT_DOUBLE_EQ(msg.pos_setpoint, 1.2);
  EXPECT_DOUBLE_EQ(msg.vel_setpoint, -0.5);
  EXPECT_DOUBLE_EQ(msg.torque_ff, 3.4);
  EXPECT_DOUBLE_EQ(msg.kp, 40.0);
  EXPECT_DOUBLE_EQ(msg.kd, 2.5);
}

TEST(LegController, InitOverloadsStoreExpectedGains) {
  auto node = makeNode("leg_controller_init_test");
  TestLegController controller(node);

  controller.init(10.0, 1.0);
  EXPECT_EQ(controller.stanceKp(), std::vector<double>({10.0, 10.0, 10.0}));
  EXPECT_EQ(controller.stanceKd(), std::vector<double>({1.0, 1.0, 1.0}));
  EXPECT_EQ(controller.swingKp(), std::vector<double>({10.0, 10.0, 10.0}));
  EXPECT_EQ(controller.swingKd(), std::vector<double>({1.0, 1.0, 1.0}));

  const std::vector<double> stance_kp{1.0, 2.0, 3.0};
  const std::vector<double> stance_kd{4.0, 5.0, 6.0};
  const std::vector<double> swing_kp{7.0, 8.0, 9.0};
  const std::vector<double> swing_kd{10.0, 11.0, 12.0};
  const std::vector<double> swing_kp_cart{13.0, 14.0, 15.0};
  const std::vector<double> swing_kd_cart{16.0, 17.0, 18.0};

  controller.init(stance_kp, stance_kd, swing_kp, swing_kd, swing_kp_cart,
                  swing_kd_cart, "/tmp/policy.onnx", 50.0);
  EXPECT_EQ(controller.stanceKp(), stance_kp);
  EXPECT_EQ(controller.stanceKd(), stance_kd);
  EXPECT_EQ(controller.swingKp(), swing_kp);
  EXPECT_EQ(controller.swingKd(), swing_kd);
  EXPECT_EQ(controller.swingKpCart(), swing_kp_cart);
  EXPECT_EQ(controller.swingKdCart(), swing_kd_cart);
  EXPECT_EQ(controller.modelPath(), "/tmp/policy.onnx");
}

TEST(LegController, UpdateLocalPlanCachesPlanAndTimestamp) {
  auto node = makeNode("leg_controller_plan_test");
  TestLegController controller(node);
  auto plan = std::make_shared<quad_msgs::msg::RobotPlan>();
  const rclcpp::Time t_msg(12, 34, RCL_ROS_TIME);

  controller.updateLocalPlanMsg(plan, t_msg);

  EXPECT_EQ(controller.localPlan(), plan);
  EXPECT_EQ(controller.localPlanTime(), t_msg);
}

TEST(JointController, OverridesStateMachineAndClampsSelectedTorque) {
  auto node = makeNode("joint_controller_test", "joint");
  JointController controller(node, "robot_1", nullptr);

  EXPECT_TRUE(controller.overrideStateMachine());

  auto cmd = std::make_shared<geometry_msgs::msg::Vector3>();
  cmd->x = 2.0;
  cmd->y = 1.0;
  cmd->z = 9.0;
  controller.updateSingleJointCommand(cmd);

  quad_msgs::msg::RobotState state;
  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  ASSERT_TRUE(controller.computeLegCommandArray(state, leg_commands, grfs));

  ASSERT_EQ(leg_commands.leg_commands.size(), 4u);
  for (size_t leg = 0; leg < leg_commands.leg_commands.size(); ++leg) {
    ASSERT_EQ(leg_commands.leg_commands[leg].motor_commands.size(), 3u);
    for (size_t joint = 0;
         joint < leg_commands.leg_commands[leg].motor_commands.size();
         ++joint) {
      const auto& motor = leg_commands.leg_commands[leg].motor_commands[joint];
      EXPECT_DOUBLE_EQ(motor.pos_setpoint, 0.0);
      EXPECT_DOUBLE_EQ(motor.vel_setpoint, 0.0);
      EXPECT_DOUBLE_EQ(motor.kp, 0.0);
      EXPECT_DOUBLE_EQ(motor.kd, 0.0);
      if (leg == 2u && joint == 1u) {
        EXPECT_DOUBLE_EQ(motor.torque_ff, 5.0);
      } else {
        EXPECT_DOUBLE_EQ(motor.torque_ff, 0.0);
      }
    }
  }
}

TEST(InverseDynamicsController, ReturnsFalseWithoutFreshLocalPlan) {
  auto node = makeNode("inverse_dynamics_no_plan_test");
  InverseDynamicsController controller(node, "robot_1", nullptr);

  quad_msgs::msg::RobotState state;
  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;

  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));
}

TEST(InverseDynamicsController, ComputesCommandsForFreshGo2Plan) {
  auto node = makeNode("inverse_dynamics_valid_plan_test");
  auto kinematics = makeGo2Kinematics(node);
  InverseDynamicsController controller(node, "robot_1", kinematics);
  controller.init({10.0, 11.0, 12.0}, {1.0, 1.1, 1.2},
                  {20.0, 21.0, 22.0}, {2.0, 2.1, 2.2},
                  {5.0, 5.0, 5.0}, {0.5, 0.5, 0.5});

  auto state = makeGo2RobotState(*kinematics);
  quad_utils::updateDynamics(*kinematics, state);
  controller.updateLocalPlanMsg(makeFreshGo2Plan(node, *kinematics),
                                node->now());

  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  ASSERT_TRUE(controller.computeLegCommandArray(state, leg_commands, grfs));

  expectFourLegCommands(leg_commands);
  ASSERT_EQ(grfs.vectors.size(), 4u);
  EXPECT_NEAR(leg_commands.leg_commands[0].motor_commands[0].kp, 10.0, 1e-9);
  EXPECT_NEAR(leg_commands.leg_commands[0].motor_commands[0].kd, 1.0, 1e-9);
}

TEST(InverseDynamicsController, RejectsStaleAndOutOfWindowPlans) {
  auto node = makeNode("inverse_dynamics_stale_plan_test");
  auto kinematics = makeGo2Kinematics(node);
  InverseDynamicsController controller(node, "robot_1", kinematics);
  controller.init({10.0, 10.0, 10.0}, {1.0, 1.0, 1.0},
                  {20.0, 20.0, 20.0}, {2.0, 2.0, 2.0},
                  {5.0, 5.0, 5.0}, {0.5, 0.5, 0.5});

  auto state = makeGo2RobotState(*kinematics);
  quad_utils::updateDynamics(*kinematics, state);
  auto plan = makeFreshGo2Plan(node, *kinematics);
  plan->header.stamp = node->now() - rclcpp::Duration::from_seconds(0.2);
  controller.updateLocalPlanMsg(plan, node->now());

  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));

  plan = makeFreshGo2Plan(node, *kinematics);
  plan->state_timestamp = node->now() - rclcpp::Duration::from_seconds(1.0);
  controller.updateLocalPlanMsg(plan, node->now());
  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));
}

TEST(UnderbrushInverseDynamicsController, ReturnsFalseWithoutPlanOrForceData) {
  auto node = makeNode("underbrush_no_plan_test", "underbrush");
  UnderbrushInverseDynamicsController controller(node, "robot_1", nullptr);
  controller.setUnderbrushParams(15.0, 2.0, 3.0, 3.0, 0.1, 0.135, 0.04);

  quad_msgs::msg::RobotState state;
  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;

  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));
}

TEST(UnderbrushInverseDynamicsController,
     ComputesCommandsForFreshGo2PlanAndBodyForces) {
  auto node = makeNode("underbrush_valid_plan_test", "underbrush");
  auto kinematics = makeGo2Kinematics(node);
  UnderbrushInverseDynamicsController controller(node, "robot_1", kinematics);
  controller.init({10.0, 10.0, 10.0}, {1.0, 1.0, 1.0},
                  {20.0, 20.0, 20.0}, {2.0, 2.0, 2.0},
                  {5.0, 5.0, 5.0}, {0.5, 0.5, 0.5});
  controller.setUnderbrushParams(15.0, 2.0, 3.0, 3.0, 0.1, 0.135, 0.04);

  auto state = makeGo2RobotState(*kinematics);
  quad_utils::updateDynamics(*kinematics, state);
  controller.updateLocalPlanMsg(makeFreshGo2Plan(node, *kinematics),
                                node->now());

  auto force = std::make_shared<quad_msgs::msg::BodyForceEstimate>();
  force->joint_torques.resize(12, 0.0);
  controller.updateBodyForceEstimate(force);

  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  ASSERT_TRUE(controller.computeLegCommandArray(state, leg_commands, grfs));
  expectFourLegCommands(leg_commands);
  EXPECT_EQ(grfs.vectors.size(), 4u);
}

TEST(InertiaEstimationController, ComputesCommandsForFreshGo2Plan) {
  auto node = makeNode("inertia_estimation_valid_plan_test",
                       "inertia_estimation");
  auto kinematics = makeGo2Kinematics(node);
  InertiaEstimationController controller(node, "robot_1", kinematics);
  controller.init({10.0, 10.0, 10.0}, {1.0, 1.0, 1.0},
                  {20.0, 21.0, 22.0}, {2.0, 2.1, 2.2},
                  {5.0, 5.0, 5.0}, {0.5, 0.5, 0.5});

  auto state = makeGo2RobotState(*kinematics);
  quad_utils::updateDynamics(*kinematics, state);
  controller.updateLocalPlanMsg(makeFreshGo2Plan(node, *kinematics),
                                node->now());

  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  ASSERT_TRUE(controller.computeLegCommandArray(state, leg_commands, grfs));

  expectFourLegCommands(leg_commands);
  EXPECT_EQ(grfs.vectors.size(), 4u);
  EXPECT_NEAR(leg_commands.leg_commands[0].motor_commands[0].kp, 20.0, 1e-9);
  EXPECT_NEAR(leg_commands.leg_commands[0].motor_commands[0].kd, 2.0, 1e-9);
}

TEST(InertiaEstimationController, RejectsMissingStaleAndOutOfWindowPlans) {
  auto node = makeNode("inertia_estimation_reject_plan_test",
                       "inertia_estimation");
  auto kinematics = makeGo2Kinematics(node);
  InertiaEstimationController controller(node, "robot_1", kinematics);
  controller.init({10.0, 10.0, 10.0}, {1.0, 1.0, 1.0},
                  {20.0, 20.0, 20.0}, {2.0, 2.0, 2.0},
                  {5.0, 5.0, 5.0}, {0.5, 0.5, 0.5});

  auto state = makeGo2RobotState(*kinematics);
  quad_utils::updateDynamics(*kinematics, state);
  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));

  auto plan = makeFreshGo2Plan(node, *kinematics);
  plan->header.stamp = node->now() - rclcpp::Duration::from_seconds(0.2);
  controller.updateLocalPlanMsg(plan, node->now());
  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));

  plan = makeFreshGo2Plan(node, *kinematics);
  plan->state_timestamp = node->now() - rclcpp::Duration::from_seconds(1.0);
  controller.updateLocalPlanMsg(plan, node->now());
  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));
}

TEST(GrfPidController, ComputesFiniteCommandsForGo2State) {
  auto node = makeNode("grf_pid_valid_state_test", "grf_pid");
  auto kinematics = makeGo2Kinematics(node);
  GrfPidController controller(node, "robot_1", kinematics);

  auto state = makeGo2RobotState(*kinematics);
  quad_utils::updateDynamics(*kinematics, state);

  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;
  ASSERT_TRUE(controller.computeLegCommandArray(state, leg_commands, grfs));

  expectFourLegCommands(leg_commands);
  ASSERT_EQ(grfs.vectors.size(), 4u);
  for (const auto& grf : grfs.vectors) {
    EXPECT_GE(grf.z, 0.0);
    EXPECT_LE(std::hypot(grf.x, grf.y), 0.2 * grf.z + 1e-9);
  }
}

TEST(StateEstimator, ReadsImuAndNormalizesQuaternion) {
  auto node = makeNode("state_estimator_imu_test");
  TestStateEstimator estimator(node);

  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  imu->linear_acceleration.x = 1.0;
  imu->linear_acceleration.y = 2.0;
  imu->linear_acceleration.z = 3.0;
  imu->angular_velocity.x = 4.0;
  imu->angular_velocity.y = 5.0;
  imu->angular_velocity.z = 6.0;
  imu->orientation.w = 2.0;
  imu->orientation.x = 0.0;
  imu->orientation.y = 0.0;
  imu->orientation.z = 0.0;

  Eigen::Vector3d linear_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation;
  estimator.readIMU(imu, linear_accel, angular_vel, orientation);

  EXPECT_TRUE(linear_accel.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(angular_vel.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  EXPECT_DOUBLE_EQ(orientation.w(), 1.0);
  EXPECT_DOUBLE_EQ(orientation.x(), 0.0);
  EXPECT_DOUBLE_EQ(orientation.y(), 0.0);
  EXPECT_DOUBLE_EQ(orientation.z(), 0.0);
}

TEST(StateEstimator, ReadsJointEncoderAndCachesSensorMessages) {
  auto node = makeNode("state_estimator_joint_test");
  TestStateEstimator estimator(node);

  auto joints = std::make_shared<sensor_msgs::msg::JointState>();
  joints->position.resize(12);
  for (size_t i = 0; i < joints->position.size(); ++i) {
    joints->position[i] = static_cast<double>(i) * 0.25;
  }

  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(12);
  estimator.readJointEncoder(joints, joint_positions);
  for (int i = 0; i < joint_positions.size(); ++i) {
    EXPECT_DOUBLE_EQ(joint_positions[i], static_cast<double>(i) * 0.25);
  }

  auto mocap = std::make_shared<geometry_msgs::msg::PoseStamped>();
  mocap->pose.position.x = 1.5;
  estimator.loadMocapMsg(mocap);
  ASSERT_NE(estimator.lastMocap(), nullptr);
  EXPECT_DOUBLE_EQ(estimator.lastMocap()->pose.position.x, 1.5);

  quad_msgs::msg::FootContact contact;
  contact.contact_states = {true, false, true, false};
  contact.foot_force_raw = {31, 12, 45, 3};
  estimator.loadFootContactMsg(contact);
  EXPECT_TRUE(estimator.footContactReceived());
  EXPECT_EQ(estimator.lastFootContact().contact_states, contact.contact_states);
  EXPECT_EQ(estimator.lastFootContact().foot_force_raw, contact.foot_force_raw);
}

TEST(CompFilterEstimator, ReturnsFalseUntilMocapIsAvailable) {
  auto node = makeNode("comp_filter_no_mocap_test", "inverse_dynamics",
                       "comp_filter");
  CompFilterEstimator estimator(node, "robot_1", nullptr);
  estimator.init();

  sensor_msgs::msg::Imu imu;
  imu.orientation.w = 1.0;
  sensor_msgs::msg::JointState joints;
  joints.position.resize(12, 0.0);
  estimator.loadSensorMsg(imu, joints);

  quad_msgs::msg::RobotState state;
  EXPECT_FALSE(estimator.updateOnce(state));
}

TEST(CompFilterEstimator, UpdatesGo2StateWhenMocapIsAvailable) {
  auto node = makeNode("comp_filter_success_test", "inverse_dynamics",
                       "comp_filter");
  auto kinematics = makeGo2Kinematics(node);
  CompFilterEstimator estimator(node, "robot_1", kinematics);
  estimator.init();

  sensor_msgs::msg::Imu imu;
  imu.orientation.w = 1.0;
  imu.linear_acceleration.z = 9.81;
  sensor_msgs::msg::JointState joints;
  joints.name = kinematics->getOrderedJointNames();
  joints.position = {0.0, 0.6, -1.2, 0.0, 0.6, -1.2,
                     0.0, 0.6, -1.2, 0.0, 0.6, -1.2};
  joints.velocity.resize(12, 0.0);
  joints.effort.resize(12, 0.0);
  estimator.loadSensorMsg(imu, joints);

  auto mocap = std::make_shared<geometry_msgs::msg::PoseStamped>();
  mocap->pose.position.x = 1.0;
  mocap->pose.position.y = -2.0;
  mocap->pose.position.z = 0.35;
  mocap->pose.orientation.w = 1.0;
  estimator.loadMocapMsg(mocap);

  quad_msgs::msg::RobotState state;
  ASSERT_TRUE(estimator.updateOnce(state));
  EXPECT_NEAR(state.body.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(state.body.pose.position.y, -2.0, 1e-9);
  EXPECT_NEAR(state.body.pose.position.z, 0.35, 1e-9);
  EXPECT_EQ(state.header.frame_id, "map");
  ASSERT_EQ(state.joints.position.size(), 12u);
  ASSERT_EQ(state.feet.feet.size(), 4u);
}

TEST(EKFEstimator, StateSettersAndMathHelpersAreConsistent) {
  auto node = makeNode("ekf_math_test", "inverse_dynamics", "ekf_filter");
  EKFEstimator estimator(node, "robot_1", nullptr);

  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(EKFEstimator::num_state, 0.0,
                                                 17.0);
  estimator.setX(x);
  estimator.setP(Eigen::MatrixXd::Identity(EKFEstimator::num_cov,
                                           EKFEstimator::num_cov));
  EXPECT_TRUE(estimator.getX().isApprox(x));

  Eigen::VectorXd w(3);
  w << 1.0, 2.0, 3.0;
  Eigen::MatrixXd skew = estimator.calcSkewsym(w);
  EXPECT_NEAR(skew(0, 1), -3.0, 1e-9);
  EXPECT_NEAR(skew(0, 2), 2.0, 1e-9);
  EXPECT_NEAR(skew(1, 0), 3.0, 1e-9);
  EXPECT_NEAR(skew(2, 1), 1.0, 1e-9);

  EXPECT_TRUE(estimator.calcRodrigues(0.0, w, 0).isApprox(
      Eigen::MatrixXd::Identity(3, 3)));

  Eigen::VectorXd q(4);
  q << 1.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd q_next = estimator.quaternionDynamics(
      Eigen::VectorXd::Zero(3), q);
  EXPECT_NEAR(q_next[0], 1.0, 1e-9);
  EXPECT_NEAR(q_next.tail<3>().norm(), 0.0, 1e-9);
}

TEST(EKFEstimator, UpdatesGo2StateFromSensorInputs) {
  auto node = makeNode("ekf_update_test", "inverse_dynamics", "ekf_filter");
  auto kinematics = makeGo2Kinematics(node);
  EKFEstimator estimator(node, "robot_1", kinematics);
  estimator.init();

  sensor_msgs::msg::Imu imu;
  imu.orientation.w = 1.0;
  imu.linear_acceleration.z = 9.81;
  sensor_msgs::msg::JointState joints;
  joints.name = kinematics->getOrderedJointNames();
  joints.position = {0.0, 0.6, -1.2, 0.0, 0.6, -1.2,
                     0.0, 0.6, -1.2, 0.0, 0.6, -1.2};
  joints.velocity.resize(12, 0.0);
  joints.effort.resize(12, 0.0);
  estimator.loadSensorMsg(imu, joints);

  auto state = makeGo2RobotState(*kinematics);
  ASSERT_TRUE(estimator.updateOnce(state));
  EXPECT_EQ(state.header.frame_id, "map");
  ASSERT_EQ(state.joints.position.size(), 12u);
  ASSERT_EQ(state.feet.feet.size(), 4u);
  EXPECT_TRUE(std::isfinite(state.body.pose.position.z));
}

TEST(RobotDriver, ConstructsWithExistingConfigsForSupportedControllers) {
  for (const char* controller :
       {"inverse_dynamics", "joint", "grf_pid", "underbrush",
        "inertia_estimation"}) {
    const std::string controller_name(controller);
    auto node = makeNode("robot_driver_" + controller_name + "_test",
                         controller_name);
    EXPECT_NO_THROW({ RobotDriver robot_driver(node, my_argc, my_argv); });
  }
}

TEST(RobotDriver, InvalidControllerDoesNotDereferenceNullController) {
  auto node = makeNode("robot_driver_invalid_controller_test", "invalid");
  RobotDriver robot_driver(node, my_argc, my_argv);

  EXPECT_FALSE(robot_driver.leg_controller_);
}

#ifndef HAS_ONNXRUNTIME
TEST(RobotDriver, LearnedControllerWithoutOnnxLeavesControllerNull) {
  auto node = makeNode("robot_driver_learned_no_onnx_test", "learned");
  RobotDriver robot_driver(node, my_argc, my_argv);

  EXPECT_FALSE(robot_driver.leg_controller_);
}
#endif

TEST(RobotDriver, ControlModeTransitionsRespectSitReadyAndSafety) {
  auto node = makeNode("robot_driver_mode_transition_test", "joint");
  RobotDriver robot_driver(node, my_argc, my_argv);
  robot_driver.last_robot_state_msg_ = makeGo2RobotState(*robot_driver.quadKD2_);
  robot_driver.last_robot_state_msg_.header.stamp = node->now();

  auto mode = std::make_shared<std_msgs::msg::UInt8>();
  mode->data = robot_driver.READY;
  robot_driver.controlModeCallback(mode);
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.SIT_TO_READY);

  mode->data = robot_driver.SIT;
  robot_driver.controlModeCallback(mode);
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.SIT_TO_READY);

  robot_driver.transition_timestamp_ =
      node->now() - rclcpp::Duration::from_seconds(2.0);
  EXPECT_TRUE(robot_driver.updateControl());
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.READY);

  mode->data = robot_driver.SIT;
  robot_driver.controlModeCallback(mode);
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.READY_TO_SIT);

  robot_driver.transition_timestamp_ =
      node->now() - rclcpp::Duration::from_seconds(2.0);
  EXPECT_TRUE(robot_driver.updateControl());
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.SIT);

  mode->data = robot_driver.SAFETY;
  robot_driver.controlModeCallback(mode);
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.SAFETY);
}

TEST(RobotDriver, SafetyChecksTripOnHeartbeatAndStateTimeouts) {
  auto node = makeNode("robot_driver_safety_timeout_test", "joint");
  RobotDriver robot_driver(node, my_argc, my_argv);
  robot_driver.control_mode_ = robot_driver.READY;
  robot_driver.heartbeat_timeout_ = 0.01;
  robot_driver.remote_heartbeat_received_time_ = node->now().seconds() - 1.0;
  robot_driver.checkMessagesForSafety();
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.SAFETY);

  robot_driver.control_mode_ = robot_driver.READY;
  robot_driver.remote_heartbeat_received_time_ =
      std::numeric_limits<double>::max();
  robot_driver.state_timeout_ = 0.01;
  robot_driver.last_state_time_ = node->now().seconds() - 1.0;
  robot_driver.checkMessagesForSafety();
  EXPECT_EQ(robot_driver.control_mode_, robot_driver.SAFETY);
}

TEST(RobotDriver, UpdateControlRejectsMissingStateAndClampsEffort) {
  auto node = makeNode("robot_driver_update_control_test", "joint");
  RobotDriver robot_driver(node, my_argc, my_argv);

  robot_driver.last_robot_state_msg_ = quad_msgs::msg::RobotState();
  EXPECT_FALSE(robot_driver.updateControl());

  robot_driver.last_robot_state_msg_.header.stamp = node->now();
  EXPECT_FALSE(robot_driver.updateControl());

  robot_driver.last_robot_state_msg_ = makeGo2RobotState(*robot_driver.quadKD2_);
  robot_driver.last_robot_state_msg_.header.stamp = node->now();
  robot_driver.control_mode_ = robot_driver.READY;
  robot_driver.torque_limits_ = {1.0, 1.0, 1.0};

  auto single_joint = std::make_shared<geometry_msgs::msg::Vector3>();
  single_joint->x = 0.0;
  single_joint->y = 0.0;
  single_joint->z = 9.0;
  robot_driver.singleJointCommandCallback(single_joint);

  ASSERT_TRUE(robot_driver.updateControl());
  const auto& motor =
      robot_driver.leg_command_array_msg_.leg_commands[0].motor_commands[0];
  EXPECT_NEAR(std::abs(motor.effort), 1.0, 1e-9);
}

TEST(RobotDriver, PublishControlAndHeartbeatPublishConfiguredTopics) {
  auto driver_node = makeNode("robot_driver_publish_test", "joint");
  RobotDriver robot_driver(driver_node, my_argc, my_argv);
  auto listener = std::make_shared<rclcpp::Node>("robot_driver_listener",
                                                "robot_1");

  auto leg_msg =
      std::make_shared<quad_msgs::msg::LegCommandArray::SharedPtr>(nullptr);
  auto grf_msg = std::make_shared<quad_msgs::msg::GRFArray::SharedPtr>(nullptr);
  auto heartbeat_msg =
      std::make_shared<std_msgs::msg::Header::SharedPtr>(nullptr);
  auto cmd_vel_msg =
      std::make_shared<geometry_msgs::msg::TwistStamped::SharedPtr>(nullptr);

  auto leg_sub = listener->create_subscription<quad_msgs::msg::LegCommandArray>(
      "control/joint_command", 10,
      [leg_msg](quad_msgs::msg::LegCommandArray::SharedPtr msg) {
        *leg_msg = msg;
      });
  auto grf_sub = listener->create_subscription<quad_msgs::msg::GRFArray>(
      "control/grfs", 10,
      [grf_msg](quad_msgs::msg::GRFArray::SharedPtr msg) { *grf_msg = msg; });
  auto heartbeat_sub = listener->create_subscription<std_msgs::msg::Header>(
      "heartbeat/robot", 10,
      [heartbeat_msg](std_msgs::msg::Header::SharedPtr msg) {
        *heartbeat_msg = msg;
      });
  auto cmd_vel_sub =
      listener->create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel_stamped", 10,
          [cmd_vel_msg](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            *cmd_vel_msg = msg;
          });

  robot_driver.leg_command_array_msg_.leg_commands.resize(4);
  for (auto& leg : robot_driver.leg_command_array_msg_.leg_commands) {
    leg.motor_commands.resize(3);
  }
  robot_driver.grf_array_msg_ = makeGrfArray();
  robot_driver.last_cmd_vel_msg_.linear.x = 1.25;

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  robot_driver.publishControl(true);
  robot_driver.last_robot_heartbeat_msg_.stamp =
      driver_node->now() - rclcpp::Duration::from_seconds(1.0);
  robot_driver.publishHeartbeat();

  EXPECT_TRUE(spinUntilMessage(driver_node, listener, *leg_msg));
  EXPECT_TRUE(spinUntilMessage(driver_node, listener, *grf_msg));
  EXPECT_TRUE(spinUntilMessage(driver_node, listener, *heartbeat_msg));
  EXPECT_TRUE(spinUntilMessage(driver_node, listener, *cmd_vel_msg));
  EXPECT_NEAR((*cmd_vel_msg)->twist.linear.x, 1.25, 1e-9);
}

TEST(UnitreeInterface, DefaultsToGo2AndRejectsRecvBeforeState) {
  TestUnitreeInterface unitree("unknown_robot");
  EXPECT_EQ(unitree.robotName(), "go2");
  EXPECT_EQ(unitree.numMotors(), 12);
  EXPECT_FALSE(unitree.hasWheels());

  sensor_msgs::msg::JointState joints;
  joints.name.resize(12);
  joints.position.resize(12);
  joints.velocity.resize(12);
  joints.effort.resize(12);
  sensor_msgs::msg::Imu imu;
  Eigen::VectorXd user_rx;
  EXPECT_FALSE(unitree.recv(joints, imu, user_rx));
}

TEST(UnitreeInterface, ReordersFootForcesIntoQuadSdkLegOrder) {
  TestUnitreeInterface unitree("go2");
  unitree_go::msg::dds_::LowState_ low_state;
  low_state.foot_force()[0] = 10;
  low_state.foot_force()[1] = 20;
  low_state.foot_force()[2] = 30;
  low_state.foot_force()[3] = 40;

  unitree.lowStateHandler(&low_state);
  const auto foot_forces = unitree.getFootForcesRaw();
  EXPECT_EQ(foot_forces[0], 20);
  EXPECT_EQ(foot_forces[1], 40);
  EXPECT_EQ(foot_forces[2], 10);
  EXPECT_EQ(foot_forces[3], 30);
}

TEST(UnitreeInterface, CrcIsDeterministic) {
  TestUnitreeInterface unitree("go2");
  uint32_t data[] = {0x12345678, 0x90abcdef, 0x00000001};

  const uint32_t first = unitree.crc32Core(data, 3);
  const uint32_t second = unitree.crc32Core(data, 3);
  EXPECT_EQ(first, second);
  EXPECT_NE(first, 0u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  my_argc = argc;
  my_argv = argv;
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
