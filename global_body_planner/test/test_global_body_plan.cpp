#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "global_body_planner/gbpl.hpp"
#include "global_body_planner/global_body_plan.hpp"
#include "global_body_planner/global_body_planner_test_fixture.hpp"
#include "test_helpers.hpp"

using global_body_planner_test::kTol;
using global_body_planner_test::makeState;

TEST(GlobalBodyPlannerPlanTest, ClearAndInvalidateResetPlanState) {
  GlobalBodyPlan plan;

  EXPECT_TRUE(plan.isEmpty());
  EXPECT_EQ(plan.getStatus(), UNSOLVED);

  plan.invalidate();

  EXPECT_EQ(plan.getStatus(), UNSOLVED);
  EXPECT_DOUBLE_EQ(plan.getLength(), std::numeric_limits<double>::max());

  plan.clear();

  EXPECT_TRUE(plan.isEmpty());
  EXPECT_EQ(plan.getStatus(), UNSOLVED);
  EXPECT_DOUBLE_EQ(plan.getLength(), 0.0);
}

TEST_F(GlobalBodyPlannerTestFixture, LoadPlanDataAndConvertToMsgs) {
  planning_utils::State s1 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s2 = s1;
  s2.pos[0] += 1.0;

  planning_utils::StateActionResult result;
  GBPL gbpl;
  ASSERT_EQ(gbpl.attemptConnect(s1, s2, 2.0, result, planner_config_, FORWARD),
            REACHED);

  std::vector<planning_utils::State> states{s1, s2};
  std::vector<planning_utils::Action> actions{result.a_new};
  planning_utils::FullState start_state =
      planning_utils::stateToFullState(s1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  GlobalBodyPlan plan;
  plan.loadPlanData(VALID, start_state, 0.0, states, actions, 0.03, 0.0,
                    planner_config_);

  EXPECT_EQ(plan.getStatus(), VALID);
  EXPECT_GT(plan.getSize(), 0);
  EXPECT_NEAR(plan.getLength(), 1.0, kTol);
  EXPECT_DOUBLE_EQ(plan.getGoalDistance(), 0.0);

  for (int i = 1; i < plan.getSize(); ++i) {
    EXPECT_GE(plan.getTime(i), plan.getTime(i - 1));
    EXPECT_GE(plan.getLengthAtIndex(i), plan.getLengthAtIndex(i - 1));
  }

  quad_msgs::msg::RobotPlan robot_plan_msg;
  quad_msgs::msg::RobotPlan discrete_plan_msg;
  robot_plan_msg.header.frame_id = "map";
  discrete_plan_msg.header.frame_id = "map";
  plan.convertToMsg(robot_plan_msg, discrete_plan_msg);

  EXPECT_EQ(robot_plan_msg.states.size(), robot_plan_msg.grfs.size());
  EXPECT_EQ(robot_plan_msg.states.size(), robot_plan_msg.plan_indices.size());
  EXPECT_EQ(robot_plan_msg.states.size(), robot_plan_msg.primitive_ids.size());
  EXPECT_EQ(discrete_plan_msg.states.size(), states.size());
}
