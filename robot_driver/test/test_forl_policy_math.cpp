#include <gtest/gtest.h>

#include "robot_driver/controllers/forl_policy_math.hpp"

namespace {

forl::Vector12d iota() {
  forl::Vector12d v;
  for (int i = 0; i < forl::kNumJoints; ++i) v(i) = i;
  return v;
}

}  // namespace

// Quad-SDK order is FL, RL, FR, RR; the policy's is FR, FL, RR, RL.
TEST(ForlPolicyMath, quadToPolicyOrderMovesWholeLegs) {
  const forl::Vector12d policy = forl::quadToPolicyOrder(iota());
  forl::Vector12d expected;
  expected << 6, 7, 8,  // FR is Quad-SDK leg 2
      0, 1, 2,          // FL is Quad-SDK leg 0
      9, 10, 11,        // RR is Quad-SDK leg 3
      3, 4, 5;          // RL is Quad-SDK leg 1
  EXPECT_TRUE(policy.isApprox(expected));
}

TEST(ForlPolicyMath, orderMappingsAreInverses) {
  const forl::Vector12d v = iota();
  EXPECT_TRUE(forl::policyToQuadOrder(forl::quadToPolicyOrder(v)).isApprox(v));
  EXPECT_TRUE(forl::quadToPolicyOrder(forl::policyToQuadOrder(v)).isApprox(v));
}

TEST(ForlPolicyMath, projectedGravityUprightAndPitched) {
  EXPECT_TRUE(forl::projectedGravity(Eigen::Quaterniond::Identity())
                  .isApprox(Eigen::Vector3d(0, 0, -1)));
  // Nose down by 90 degrees (positive pitch about +y): gravity points along
  // the body's +x axis.
  const Eigen::Quaterniond nose_down(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(forl::projectedGravity(nose_down)
                  .isApprox(Eigen::Vector3d(1, 0, 0), 1e-9));
  // Rolled right side down by 90 degrees (positive roll about +x): gravity
  // points along the body's -y axis.
  const Eigen::Quaterniond rolled(
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  EXPECT_TRUE(forl::projectedGravity(rolled).isApprox(
      Eigen::Vector3d(0, -1, 0), 1e-9));
}

TEST(ForlPolicyMath, clipCommandIsSymmetric) {
  const Eigen::Vector3d limits(1.0, 0.4, 1.0);
  EXPECT_TRUE(forl::clipCommand(Eigen::Vector3d(2.0, -0.9, 0.3), limits)
                  .isApprox(Eigen::Vector3d(1.0, -0.4, 0.3)));
}

TEST(ForlPolicyMath, observationLayout) {
  const forl::Vector12d default_pose =
      forl::defaultPose(Eigen::Vector3d(0.0, 0.9, -1.8));
  // Robot standing exactly at the default pose, in Quad-SDK order.
  const forl::Vector12d q_quad = forl::policyToQuadOrder(default_pose);
  forl::Vector12d qd_quad = forl::Vector12d::Zero();
  qd_quad(6) = 3.0;  // Quad-SDK FR abad -> policy joint 0
  forl::Vector12d prev_action = forl::Vector12d::Constant(0.25);
  const forl::ObsVector obs = forl::assembleObservation(
      Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0, 0, -1), q_quad,
      qd_quad, prev_action, Eigen::Vector3d(0.5, -0.1, 0.2), default_pose);

  EXPECT_FLOAT_EQ(obs(0), 0.1f);   // angular velocity 0..2
  EXPECT_FLOAT_EQ(obs(2), 0.3f);
  EXPECT_FLOAT_EQ(obs(5), -1.0f);  // gravity 3..5
  for (int i = 6; i < 18; ++i) EXPECT_NEAR(obs(i), 0.0f, 1e-6);  // q - default
  EXPECT_FLOAT_EQ(obs(18), 3.0f);  // joint velocity 18..29
  for (int i = 19; i < 30; ++i) EXPECT_FLOAT_EQ(obs(i), 0.0f);
  for (int i = 30; i < 42; ++i) EXPECT_FLOAT_EQ(obs(i), 0.25f);  // prev action
  EXPECT_FLOAT_EQ(obs(42), 0.5f);  // command 42..44
  EXPECT_FLOAT_EQ(obs(43), -0.1f);
  EXPECT_FLOAT_EQ(obs(44), 0.2f);
}

TEST(ForlPolicyMath, actionToTargets) {
  const forl::Vector12d default_pose =
      forl::defaultPose(Eigen::Vector3d(0.0, 0.9, -1.8));
  // Zero action holds the default pose on every leg.
  const forl::Vector12d hold =
      forl::actionToTargets(forl::Vector12d::Zero(), default_pose, 0.5);
  for (int leg = 0; leg < 4; ++leg) {
    EXPECT_DOUBLE_EQ(hold(3 * leg + 0), 0.0);
    EXPECT_DOUBLE_EQ(hold(3 * leg + 1), 0.9);
    EXPECT_DOUBLE_EQ(hold(3 * leg + 2), -1.8);
  }
  // Policy joint 1 is the FR thigh, Quad-SDK joint 7. Actions beyond +-1 are
  // clipped.
  forl::Vector12d action = forl::Vector12d::Zero();
  action(1) = 1.0;
  action(5) = -3.0;  // FL calf -> Quad-SDK joint 2
  const forl::Vector12d targets =
      forl::actionToTargets(action, default_pose, 0.5);
  EXPECT_DOUBLE_EQ(targets(7), 0.9 + 0.5);
  EXPECT_DOUBLE_EQ(targets(2), -1.8 - 0.5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
