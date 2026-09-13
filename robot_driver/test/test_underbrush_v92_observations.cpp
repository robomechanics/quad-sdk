#include <gtest/gtest.h>
#include "robot_driver/controllers/underbrush_policy.hpp"

// Exercise observation assembly only: no ONNX load, robot driver, or hardware.
class ObservationProbe : public UnderbrushPolicy {
 public:
  ObservationProbe(rclcpp::Node::SharedPtr node, bool v92)
      : UnderbrushPolicy(node, "test", nullptr, v92) {
    cmd_vel_msg_ = Eigen::VectorXd::Zero(6);
  }
  const auto& legs() const { return obs_per_leg_; }
  const auto& body() const { return obs_body_; }
  const auto& hidden() const { return h_state_; }
  const auto& rawActions() const { return raw_actions_; }
  void setPreviousAction() {
    for (int i = 0; i < 12; ++i) raw_actions_(i) = 0.25 * (i + 1);
  }
};

TEST(UnderbrushV92, InitialZeroHistoryAndJointOrder) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("underbrush_observation_test");
  ObservationProbe v90(node, false), v92(node, true);
  quad_msgs::msg::RobotState state;
  state.body.pose.orientation.w = 1.0;
  for (int i = 0; i < 12; ++i) {
    state.joints.position.push_back(0.01 * (i + 1));
    state.joints.velocity.push_back(0.1 * (i + 1));
    state.joints.effort.push_back(-2.0 * (i + 1));
  }
  quad_msgs::msg::FootContact contacts;
  contacts.contact_states = {true, false, false, true};
  v90.updateFootContactMsg(contacts);
  v92.updateFootContactMsg(contacts);
  v92.computeObservations(state);
  for (int leg = 0; leg < 4; ++leg) {
    ASSERT_EQ(v92.legs()[leg].size(), 13u);
    for (int j = 0; j < 3; ++j) EXPECT_FLOAT_EQ(v92.legs()[leg][10+j], 0.0f);
  }
  v90.setPreviousAction();
  v92.setPreviousAction();
  v90.computeObservations(state);
  v92.computeObservations(state);
  EXPECT_EQ(v90.body(), v92.body());
  const int quad_leg[] = {0, 2, 1, 3};
  for (int leg = 0; leg < 4; ++leg) {
    ASSERT_EQ(v90.legs()[leg].size(), 10u);
    EXPECT_FLOAT_EQ(v92.legs()[leg][9], contacts.contact_states[quad_leg[leg]] ? 1.f : 0.f);
    for (int i = 0; i < 10; ++i) EXPECT_FLOAT_EQ(v90.legs()[leg][i], v92.legs()[leg][i]);
    for (int j = 0; j < 3; ++j) {
      EXPECT_FLOAT_EQ(v92.legs()[leg][j], 0.01f * (3 * quad_leg[leg] + j + 1));
      EXPECT_FLOAT_EQ(v92.legs()[leg][6+j], -0.02f * (3 * quad_leg[leg] + j + 1));
      EXPECT_FLOAT_EQ(v92.legs()[leg][10+j], 0.25f * (4*j + leg + 1));
    }
  }
  rclcpp::shutdown();
}

TEST(UnderbrushV90, CpuPreflightPreservesInitialStateAndRejectsV92Mismatch) {
  const char* model = std::getenv("UNDERBRUSH_TEST_MODEL");
  if (!model) GTEST_SKIP() << "Set UNDERBRUSH_TEST_MODEL to the V90 ONNX export";
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("underbrush_cpu_init_test");
  ObservationProbe v90(node, false), v92(node, true);
  const std::vector<double> kp{25,25,25}, kd{.5,.5,.5}, zeros{0,0,0}, stance{0,.8,-1.5};
  ASSERT_NO_THROW(v90.init(kp,kd,kp,kd,zeros,zeros,model,50,stance));
  for (const auto& hidden : v90.hidden())
    for (float value : hidden) EXPECT_FLOAT_EQ(value, 0.f);
  EXPECT_DOUBLE_EQ(v90.rawActions().norm(), 0.0);
  EXPECT_THROW(v92.init(kp,kd,kp,kd,zeros,zeros,model,50,stance), std::runtime_error);
  rclcpp::shutdown();
}
