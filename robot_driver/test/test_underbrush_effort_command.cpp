#include <gtest/gtest.h>
#include <limits>
#include "robot_driver/controllers/underbrush_effort_command.hpp"
#include "robot_driver/hardware_interfaces/unitree_interface.hpp"

namespace {
quad_msgs::msg::MotorCommand policyCommand(double target) {
  quad_msgs::msg::MotorCommand cmd;
  cmd.pos_setpoint = target;
  cmd.kp = 25.0f;
  cmd.kd = 0.5f;
  return cmd;
}

// Exercise the exact SDK packet builder, without DDS, ROS nodes or hardware.
class InterfaceProbe : public UnitreeInterface {
 public:
  InterfaceProbe() : UnitreeInterface("go2") { initLowCmd(); }
  void prepare(const quad_msgs::msg::LegCommandArray& commands) {
    prepareCommand(commands, Eigen::VectorXd::Zero(1));
  }
  const auto& motors() const { return low_cmd_.motor_cmd(); }
  void receive() {
    unitree_go::msg::dds_::LowState_ state{};
    lowStateHandler(&state);
  }
  void expire() { last_state_received_ -= std::chrono::seconds(1); }
};
}  // namespace

TEST(UnderbrushEffort, ClipsTotalPdAndFeedforwardWithoutDoublePd) {
  auto cmd = policyCommand(2.0);
  cmd.torque_ff = 1.0;
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(cmd, 0.0, 10.0, 33.5));
  EXPECT_DOUBLE_EQ(cmd.pos_component, 50.0);
  EXPECT_DOUBLE_EQ(cmd.vel_component, -5.0);
  EXPECT_DOUBLE_EQ(cmd.fb_component, 45.0);
  EXPECT_DOUBLE_EQ(cmd.effort, 20.2);
  EXPECT_DOUBLE_EQ(cmd.torque_ff, 20.2);
  EXPECT_FLOAT_EQ(cmd.kp, 0.0f);
  EXPECT_FLOAT_EQ(cmd.kd, 0.0f);
  EXPECT_DOUBLE_EQ(cmd.pos_setpoint, 2.0);

  cmd = policyCommand(-2.0);
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(cmd, 0.0, 21.75, 50.0));
  EXPECT_DOUBLE_EQ(cmd.torque_ff, -11.7);  // opposing motion, half-speed envelope
  cmd = policyCommand(2.0);
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(cmd, 0.0, 30.0, 50.0));
  EXPECT_DOUBLE_EQ(cmd.torque_ff, 0.0);
  cmd = policyCommand(2.0);
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(cmd, 0.0, 0.0, 10.0));
  EXPECT_DOUBLE_EQ(cmd.torque_ff, 10.0);  // retain tighter configured motor cap
}

TEST(UnderbrushEffort, RecomputesFromFreshStateBetweenActorUpdates) {
  const auto target = policyCommand(0.4);
  auto first = target;
  auto next = target;
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(first, 0.0, 0.0, 33.5));
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(next, 0.2, 2.0, 33.5));
  EXPECT_DOUBLE_EQ(first.torque_ff, 10.0);
  EXPECT_DOUBLE_EQ(next.torque_ff, 4.0);
  EXPECT_DOUBLE_EQ(first.pos_setpoint, next.pos_setpoint);
  EXPECT_FLOAT_EQ(target.kp, 25.0f);  // original actor command is retained
}

TEST(UnderbrushEffort, RejectsInvalidInputsWithoutPartiallyConvertingCommand) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int field = 0; field < 7; ++field) {
    auto cmd = policyCommand(0.4);
    double q = 0.0, qd = 0.0, limit = 33.5;
    if (field == 0) q = nan;
    if (field == 1) qd = nan;
    if (field == 2) cmd.pos_setpoint = nan;
    if (field == 3) cmd.vel_setpoint = nan;
    if (field == 4) cmd.torque_ff = nan;
    if (field == 5) limit = 0.0;
    if (field == 6) cmd.kd = -0.5f;
    EXPECT_FALSE(underbrush::makeClippedEffortCommand(cmd, q, qd, limit));
    EXPECT_FLOAT_EQ(cmd.kp, 25.0f);
  }
  auto zero = policyCommand(0.0);
  ASSERT_TRUE(underbrush::makeClippedEffortCommand(zero, 0.0, 0.0, 33.5));
  EXPECT_DOUBLE_EQ(zero.fb_ratio, 0.0);
}

TEST(UnderbrushEffort, UnitreePacketContainsLimitedTorqueAndCorrectJointMapping) {
  InterfaceProbe interface;
  quad_msgs::msg::LegCommandArray commands;
  commands.leg_commands.resize(4);
  const int mapping[4][3] = {{3,4,5}, {9,10,11}, {0,1,2}, {6,7,8}};
  for (int leg = 0; leg < 4; ++leg) {
    auto& motors = commands.leg_commands[leg].motor_commands;
    motors.resize(3);
    for (int joint = 0; joint < 3; ++joint) {
      motors[joint] = policyCommand(0.02 * (3 * leg + joint + 1));
      ASSERT_TRUE(underbrush::makeClippedEffortCommand(motors[joint], 0.0, 0.0, 33.5));
    }
  }
  interface.prepare(commands);
  for (int leg = 0; leg < 4; ++leg) {
    for (int joint = 0; joint < 3; ++joint) {
      const auto& packet = interface.motors()[mapping[leg][joint]];
      EXPECT_FLOAT_EQ(packet.tau(), 0.5f * (3 * leg + joint + 1));
      EXPECT_FLOAT_EQ(packet.kp(), 0.0f);
      EXPECT_FLOAT_EQ(packet.kd(), 0.0f);
    }
  }
  // Stance/safety and legacy commands bypass conversion and retain onboard PD.
  for (auto& leg : commands.leg_commands)
    for (auto& cmd : leg.motor_commands) {
      cmd.kp = 60.0f; cmd.kd = 4.0f; cmd.torque_ff = 0.0;
    }
  interface.prepare(commands);
  for (int i = 0; i < 12; ++i) {
    EXPECT_FLOAT_EQ(interface.motors()[i].kp(), 60.0f);
    EXPECT_FLOAT_EQ(interface.motors()[i].kd(), 4.0f);
    EXPECT_FLOAT_EQ(interface.motors()[i].tau(), 0.0f);
  }
}

TEST(UnderbrushEffort, FreshnessRequiresRecentActualHardwareReceipt) {
  InterfaceProbe interface;
  EXPECT_FALSE(interface.hasFreshState(0.02));
  interface.receive();
  EXPECT_TRUE(interface.hasFreshState(0.5));
  interface.expire();
  EXPECT_FALSE(interface.hasFreshState(0.02));
}
