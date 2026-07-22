#include <gtest/gtest.h>

#include "quad_utils/ros_utils.hpp"

namespace {
constexpr double kTol = 1e-9;

rclcpp::Time makeTime(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

std_msgs::msg::Header makeHeader(double seconds, const std::string& frame) {
  std_msgs::msg::Header header;
  header.stamp = makeTime(seconds);
  header.frame_id = frame;
  return header;
}

quad_msgs::msg::FootState makeFoot(double x, double y, double z) {
  quad_msgs::msg::FootState foot;
  foot.position.x = x;
  foot.position.y = y;
  foot.position.z = z;
  foot.velocity.x = 2.0 * x;
  foot.velocity.y = 2.0 * y;
  foot.velocity.z = 2.0 * z;
  foot.acceleration.x = 3.0 * x;
  foot.acceleration.y = 3.0 * y;
  foot.acceleration.z = 3.0 * z;
  foot.contact = true;
  return foot;
}

quad_msgs::msg::RobotState makeRobotState(double t, double x) {
  quad_msgs::msg::RobotState state;
  state.header = makeHeader(t, "map");
  Eigen::VectorXd body(12);
  body << x, 2.0 * x, 0.5, 0.0, 0.0, 0.0, x + 1.0, x + 2.0, x + 3.0, 0.1,
      0.2, 0.3;
  state.body = quad_utils::eigenToBodyStateMsg(body);
  state.body.header = state.header;

  state.joints.header = state.header;
  state.joints.name = {"0", "1"};
  state.joints.position = {x, x + 1.0};
  state.joints.velocity = {x + 2.0, x + 3.0};
  state.joints.effort = {x + 4.0, x + 5.0};

  state.feet.header = state.header;
  state.feet.feet = {makeFoot(x, 0.0, 0.0), makeFoot(0.0, x, 0.0)};
  return state;
}

quad_msgs::msg::GRFArray makeGRF(double t, double scale) {
  quad_msgs::msg::GRFArray grf;
  grf.header = makeHeader(t, "map");
  geometry_msgs::msg::Vector3 vec;
  vec.x = scale;
  vec.y = 2.0 * scale;
  vec.z = 3.0 * scale;
  geometry_msgs::msg::Point point;
  point.x = scale + 1.0;
  point.y = scale + 2.0;
  point.z = scale + 3.0;
  grf.vectors = {vec};
  grf.points = {point};
  grf.contact_states = {scale > 0.0};
  return grf;
}
}  // namespace

TEST(RosUtilsTest, HeaderTimingAndStateHeaderPropagation) {
  auto now = makeTime(12.5);
  auto old_header = makeHeader(10.0, "map");
  EXPECT_NEAR(quad_utils::getROSMessageAgeInMs(old_header, now), 2500.0,
              kTol);

  quad_msgs::msg::RobotState state;
  state.feet.feet.resize(2);
  quad_utils::updateStateHeaders(state, now, "odom", 7);
  EXPECT_EQ(state.header.frame_id, "odom");
  EXPECT_EQ(state.body.header.frame_id, "odom");
  EXPECT_EQ(state.joints.header.frame_id, "odom");
  EXPECT_EQ(state.feet.traj_index, 7u);
  EXPECT_EQ(state.feet.feet[1].traj_index, 7u);
}

TEST(RosUtilsTest, EigenAndMessageConversionsRoundTrip) {
  Eigen::VectorXd body(12);
  body << 1.0, 2.0, 3.0, 0.1, -0.2, 0.3, 4.0, 5.0, 6.0, 0.4, 0.5, 0.6;
  const auto body_msg = quad_utils::eigenToBodyStateMsg(body);
  EXPECT_TRUE(quad_utils::bodyStateMsgToEigen(body_msg).isApprox(body, 1e-9));

  Eigen::VectorXd vec(4);
  vec << 1.0, 2.0, 3.0, 4.0;
  std::vector<double> stl;
  quad_utils::eigenToVector(vec, stl);
  EXPECT_EQ(stl, std::vector<double>({1.0, 2.0, 3.0, 4.0}));
  Eigen::VectorXd vec_back;
  quad_utils::vectorToEigen(stl, vec_back);
  EXPECT_TRUE(vec_back.isApprox(vec));

  Eigen::Vector3d xyz(1.0, -2.0, 3.5);
  geometry_msgs::msg::Vector3 vector_msg;
  quad_utils::Eigen3ToVector3Msg(xyz, vector_msg);
  Eigen::Vector3d xyz_back;
  quad_utils::vector3MsgToEigen(vector_msg, xyz_back);
  EXPECT_TRUE(xyz_back.isApprox(xyz));

  geometry_msgs::msg::Point point_msg;
  quad_utils::Eigen3ToPointMsg(xyz, point_msg);
  quad_utils::pointMsgToEigen(point_msg, xyz_back);
  EXPECT_TRUE(xyz_back.isApprox(xyz));
}

TEST(RosUtilsTest, FootAndGRFConversions) {
  quad_msgs::msg::MultiFootState feet;
  feet.feet = {makeFoot(1.0, 2.0, 3.0), makeFoot(4.0, 5.0, 6.0)};

  Eigen::VectorXd foot_positions(6), foot_velocities(6), foot_accelerations(6);
  quad_utils::multiFootStateMsgToEigen(feet, foot_positions, foot_velocities,
                                       foot_accelerations);
  Eigen::VectorXd expected_positions(6);
  expected_positions << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  EXPECT_TRUE(foot_positions.isApprox(expected_positions));
  EXPECT_TRUE(foot_velocities.isApprox(2.0 * expected_positions));
  EXPECT_TRUE(foot_accelerations.isApprox(3.0 * expected_positions));

  Eigen::VectorXd grfs(6);
  grfs << 0.0, 0.0, 0.0, 1.0, 2.0, 3.0;
  quad_msgs::msg::GRFArray grf_msg;
  quad_utils::eigenToGRFArrayMsg(grfs, feet, grf_msg);
  ASSERT_EQ(grf_msg.vectors.size(), 2u);
  EXPECT_FALSE(grf_msg.contact_states[0]);
  EXPECT_TRUE(grf_msg.contact_states[1]);
  EXPECT_TRUE(quad_utils::grfArrayMsgToEigen(grf_msg).isApprox(grfs));
}

TEST(RosUtilsTest, InterpolatesRobotPlanAndFootPlan) {
  quad_msgs::msg::RobotPlan plan;
  plan.states = {makeRobotState(10.0, 0.0), makeRobotState(12.0, 4.0)};
  plan.grfs = {makeGRF(10.0, 1.0), makeGRF(12.0, 5.0)};
  plan.primitive_ids = {3, 4};

  quad_msgs::msg::RobotState interp_state;
  quad_msgs::msg::GRFArray interp_grf;
  int primitive_id = -1;
  quad_utils::interpRobotPlan(plan, 1.0, interp_state, primitive_id,
                              interp_grf);

  EXPECT_EQ(primitive_id, 3);
  EXPECT_NEAR(interp_state.body.pose.position.x, 2.0, kTol);
  EXPECT_NEAR(interp_state.joints.position[1], 3.0, kTol);
  ASSERT_EQ(interp_grf.vectors.size(), 1u);
  EXPECT_NEAR(interp_grf.vectors[0].x, 3.0, kTol);

  quad_msgs::msg::MultiFootPlanContinuous foot_plan;
  quad_msgs::msg::MultiFootState feet0;
  feet0.header = makeHeader(2.0, "map");
  feet0.feet = {makeFoot(0.0, 0.0, 0.0)};
  quad_msgs::msg::MultiFootState feet1;
  feet1.header = makeHeader(4.0, "map");
  feet1.feet = {makeFoot(2.0, 4.0, 6.0)};
  foot_plan.states = {feet0, feet1};

  const auto interp_feet = quad_utils::interpMultiFootPlanContinuous(foot_plan,
                                                                     1.0);
  ASSERT_EQ(interp_feet.feet.size(), 1u);
  EXPECT_NEAR(interp_feet.feet[0].position.x, 1.0, kTol);
  EXPECT_NEAR(interp_feet.feet[0].velocity.z, 6.0, kTol);
}

TEST(RosUtilsTest, RobotPlanInterpolationClampsToPlanBounds) {
  quad_msgs::msg::RobotPlan plan;
  plan.states = {makeRobotState(10.0, 0.0), makeRobotState(12.0, 4.0),
                 makeRobotState(14.0, 8.0)};
  plan.grfs = {makeGRF(10.0, 1.0), makeGRF(12.0, 5.0),
               makeGRF(14.0, 9.0)};
  plan.primitive_ids = {3, 4, 5};

  quad_msgs::msg::RobotState interp_state;
  quad_msgs::msg::GRFArray interp_grf;
  int primitive_id = -1;

  quad_utils::interpRobotPlan(plan, -0.5, interp_state, primitive_id,
                              interp_grf);
  EXPECT_EQ(primitive_id, 3);
  EXPECT_NEAR(interp_state.body.pose.position.x, 0.0, kTol);
  ASSERT_EQ(interp_grf.vectors.size(), 1u);
  EXPECT_NEAR(interp_grf.vectors[0].x, 1.0, kTol);

  quad_utils::interpRobotPlan(plan, 0.0, interp_state, primitive_id,
                              interp_grf);
  EXPECT_EQ(primitive_id, 3);
  EXPECT_NEAR(interp_state.body.pose.position.x, 0.0, kTol);

  quad_utils::interpRobotPlan(plan, 4.0, interp_state, primitive_id,
                              interp_grf);
  EXPECT_EQ(primitive_id, 4);
  EXPECT_NEAR(interp_state.body.pose.position.x, 8.0, kTol);
  EXPECT_NEAR(interp_grf.vectors[0].x, 9.0, kTol);

  quad_utils::interpRobotPlan(plan, 5.0, interp_state, primitive_id,
                              interp_grf);
  EXPECT_EQ(primitive_id, 4);
  EXPECT_NEAR(interp_state.body.pose.position.x, 8.0, kTol);
  EXPECT_NEAR(interp_grf.vectors[0].x, 9.0, kTol);
}

TEST(RosUtilsTest, MultiFootPlanInterpolationClampsToPlanBounds) {
  quad_msgs::msg::MultiFootPlanContinuous foot_plan;
  quad_msgs::msg::MultiFootState feet0;
  feet0.header = makeHeader(2.0, "map");
  feet0.feet = {makeFoot(0.0, 0.0, 0.0)};
  quad_msgs::msg::MultiFootState feet1;
  feet1.header = makeHeader(4.0, "map");
  feet1.feet = {makeFoot(2.0, 4.0, 6.0)};
  quad_msgs::msg::MultiFootState feet2;
  feet2.header = makeHeader(6.0, "map");
  feet2.feet = {makeFoot(4.0, 8.0, 12.0)};
  foot_plan.states = {feet0, feet1, feet2};

  auto interp_feet =
      quad_utils::interpMultiFootPlanContinuous(foot_plan, -0.5);
  ASSERT_EQ(interp_feet.feet.size(), 1u);
  EXPECT_NEAR(interp_feet.feet[0].position.x, 0.0, kTol);

  interp_feet = quad_utils::interpMultiFootPlanContinuous(foot_plan, 0.0);
  EXPECT_NEAR(interp_feet.feet[0].position.x, 0.0, kTol);

  interp_feet = quad_utils::interpMultiFootPlanContinuous(foot_plan, 4.0);
  EXPECT_NEAR(interp_feet.feet[0].position.x, 4.0, kTol);

  interp_feet = quad_utils::interpMultiFootPlanContinuous(foot_plan, 5.0);
  EXPECT_NEAR(interp_feet.feet[0].position.x, 4.0, kTol);
}
