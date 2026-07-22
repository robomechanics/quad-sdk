#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "quad_utils/quad_kd2.hpp"
#include "robot_driver/controllers/grf_pid_controller.hpp"
#include "robot_driver/controllers/inverse_dynamics_controller.hpp"
#include "robot_driver/controllers/joint_controller.hpp"
#include "robot_driver/controllers/leg_controller.hpp"
#include "robot_driver/controllers/underbrush_inverse_dynamics.hpp"
#include "robot_driver/estimators/comp_filter_estimator.hpp"
#include "robot_driver/estimators/state_estimator.hpp"
#include "robot_driver/robot_driver.hpp"
#include "robot_driver/robot_driver_utils.hpp"

int my_argc;
char** my_argv;

static std::string minimalGo2Urdf() {
  return R"(
<robot name="test_go2">
  <link name="body"/>
  <link name="hip0"/><link name="upper0"/><link name="lower0"/><link name="toe0"/>
  <link name="hip1"/><link name="upper1"/><link name="lower1"/><link name="toe1"/>
  <link name="hip2"/><link name="upper2"/><link name="lower2"/><link name="toe2"/>
  <link name="hip3"/><link name="upper3"/><link name="lower3"/><link name="toe3"/>

  <joint name="8" type="revolute">
    <parent link="body"/><child link="hip0"/>
    <origin xyz="0.2 0.1 0"/><axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="40" velocity="40"/>
  </joint>
  <joint name="0" type="revolute">
    <parent link="hip0"/><child link="upper0"/>
    <origin xyz="0 0.05 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="40" velocity="40"/>
  </joint>
  <joint name="1" type="revolute">
    <parent link="upper0"/><child link="lower0"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="40" velocity="40"/>
  </joint>
  <joint name="toe0_fixed" type="fixed">
    <parent link="lower0"/><child link="toe0"/>
    <origin xyz="0 0 -0.2"/>
  </joint>

  <joint name="9" type="revolute">
    <parent link="body"/><child link="hip1"/>
    <origin xyz="-0.2 0.1 0"/><axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="40" velocity="40"/>
  </joint>
  <joint name="2" type="revolute">
    <parent link="hip1"/><child link="upper1"/>
    <origin xyz="0 0.05 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="40" velocity="40"/>
  </joint>
  <joint name="3" type="revolute">
    <parent link="upper1"/><child link="lower1"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="40" velocity="40"/>
  </joint>
  <joint name="toe1_fixed" type="fixed">
    <parent link="lower1"/><child link="toe1"/>
    <origin xyz="0 0 -0.2"/>
  </joint>

  <joint name="10" type="revolute">
    <parent link="body"/><child link="hip2"/>
    <origin xyz="0.2 -0.1 0"/><axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="40" velocity="40"/>
  </joint>
  <joint name="4" type="revolute">
    <parent link="hip2"/><child link="upper2"/>
    <origin xyz="0 -0.05 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="40" velocity="40"/>
  </joint>
  <joint name="5" type="revolute">
    <parent link="upper2"/><child link="lower2"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="40" velocity="40"/>
  </joint>
  <joint name="toe2_fixed" type="fixed">
    <parent link="lower2"/><child link="toe2"/>
    <origin xyz="0 0 -0.2"/>
  </joint>

  <joint name="11" type="revolute">
    <parent link="body"/><child link="hip3"/>
    <origin xyz="-0.2 -0.1 0"/><axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="40" velocity="40"/>
  </joint>
  <joint name="6" type="revolute">
    <parent link="hip3"/><child link="upper3"/>
    <origin xyz="0 -0.05 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="40" velocity="40"/>
  </joint>
  <joint name="7" type="revolute">
    <parent link="upper3"/><child link="lower3"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="40" velocity="40"/>
  </joint>
  <joint name="toe3_fixed" type="fixed">
    <parent link="lower3"/><child link="toe3"/>
    <origin xyz="0 0 -0.2"/>
  </joint>
</robot>)";
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
  options.append_parameter_override("robot_description", minimalGo2Urdf());
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

TEST(UnderbrushInverseDynamicsController, ReturnsFalseWithoutPlanOrForceData) {
  auto node = makeNode("underbrush_no_plan_test", "underbrush");
  UnderbrushInverseDynamicsController controller(node, "robot_1", nullptr);
  controller.setUnderbrushParams(15.0, 2.0, 3.0, 3.0, 0.1, 0.135, 0.04);

  quad_msgs::msg::RobotState state;
  quad_msgs::msg::LegCommandArray leg_commands;
  quad_msgs::msg::GRFArray grfs;

  EXPECT_FALSE(controller.computeLegCommandArray(state, leg_commands, grfs));
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  my_argc = argc;
  my_argv = argv;
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
