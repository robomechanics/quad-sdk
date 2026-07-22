#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "global_body_planner/global_body_planner_test_fixture.hpp"

namespace {

planning_utils::State makeState(double x, double y, double z, double vx,
                                double vy, double vz) {
  planning_utils::State s;
  s.pos << x, y, z;
  s.vel << vx, vy, vz;
  return s;
}

planning_utils::TimedPoseConstraint makeConstraint(
    const Eigen::Vector3d& pos, double yaw,
    const Eigen::Vector3d& half_extents, double t_start, double t_end) {
  planning_utils::TimedPoseConstraint c;
  c.pos = pos;
  c.yaw = yaw;
  c.half_extents = half_extents;
  c.t_start = t_start;
  c.t_end = t_end;
  return c;
}

planning_utils::Action makeConnectAction(double duration = 1.0) {
  planning_utils::Action a;
  a.grf_0 << 0.0, 0.0, 0.3;
  a.grf_f << 0.0, 0.0, 0.3;
  a.t_s_leap = duration;
  a.t_f = 0.0;
  a.t_s_land = 0.0;
  a.dz_0 = 0.0;
  a.dz_f = 0.0;
  return a;
}

class TestGBPL : public GBPL {
 public:
  bool hasCache() const { return has_cache_; }
  const PlannerClass* forwardCache() const { return Ta_cache_.get(); }
  const PlannerClass* reverseCache() const { return Tb_cache_.get(); }
};

}  // namespace

TEST(GlobalBodyPlannerGraphTest, GraphTracksCostsAndVertexTimes) {
  GraphClass graph;
  const planning_utils::State root =
      makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  const planning_utils::State child =
      makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  const planning_utils::State grandchild =
      makeState(1.0, 2.0, 0.3, 0.0, 0.0, 0.0);

  graph.init(root);
  graph.addVertex(1, child);
  graph.addEdge(0, 1, 1.5);
  graph.addVertex(2, grandchild);
  graph.addEdge(1, 2, 2.5);
  graph.setTime(1, 0.4);

  EXPECT_EQ(graph.getNumVertices(), 3);
  EXPECT_DOUBLE_EQ(graph.getGValue(0), 0.0);
  EXPECT_DOUBLE_EQ(graph.getGValue(1), 1.5);
  EXPECT_DOUBLE_EQ(graph.getGValue(2), 4.0);
  EXPECT_DOUBLE_EQ(graph.getTime(0), 0.0);
  EXPECT_DOUBLE_EQ(graph.getTime(1), 0.4);
  EXPECT_DOUBLE_EQ(graph.getTime(2), 0.0);

  graph.updateGValue(1, 3.0);

  EXPECT_DOUBLE_EQ(graph.getGValue(1), 3.0);
  EXPECT_DOUBLE_EQ(graph.getGValue(2), 5.0);
}

TEST_F(GlobalBodyPlannerTestFixture,
       PlannerClassNeighborQueriesSkipInvalidVertices) {
  PlannerClass planner(FORWARD, planner_config_);
  const planning_utils::State root =
      makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  const planning_utils::State near =
      makeState(0.2, 0.0, 0.3, 0.0, 0.0, 0.0);
  const planning_utils::State far =
      makeState(2.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  const planning_utils::State query =
      makeState(0.21, 0.0, 0.3, 0.0, 0.0, 0.0);

  planner.init(root);
  planner.addVertex(1, near);
  planner.addVertex(2, far);

  EXPECT_EQ(planner.getNearestNeighbor(query), 1);

  planner.markVertexInvalid(1);

  EXPECT_TRUE(planner.isVertexInvalid(1));
  EXPECT_NE(planner.getNearestNeighbor(query), 1);

  const std::vector<int> neighbors = planner.neighborhoodDist(query, 3.0);
  EXPECT_EQ(std::find(neighbors.begin(), neighbors.end(), 1), neighbors.end());

  planner.resetInvalidVertices();

  EXPECT_FALSE(planner.isVertexInvalid(1));
  EXPECT_EQ(planner.getNearestNeighbor(query), 1);
}

TEST_F(GlobalBodyPlannerTestFixture,
       PlannerClassPruneByConstraintsInvalidatesDescendants) {
  PlannerClass planner(FORWARD, planner_config_);
  planner.init(makeState(-1.0, 0.0, 0.3, 1.0, 0.0, 0.0));
  planner.addVertex(1, makeState(0.0, 0.0, 0.3, 1.0, 0.0, 0.0));
  planner.addEdge(0, 1, 1.0);
  planner.setTime(1, 1.5);
  planner.addVertex(2, makeState(1.0, 0.0, 0.3, 1.0, 0.0, 0.0));
  planner.addEdge(1, 2, 1.0);
  planner.setTime(2, 2.5);

  planner_config_.dynamic_constraints.push_back(makeConstraint(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, Eigen::Vector3d(0.5, 0.25, 0.2),
      1.0, 2.0));

  EXPECT_EQ(planner.pruneByConstraints(planner_config_), 2);
  EXPECT_FALSE(planner.isVertexInvalid(0));
  EXPECT_TRUE(planner.isVertexInvalid(1));
  EXPECT_TRUE(planner.isVertexInvalid(2));
}

TEST_F(GlobalBodyPlannerTestFixture, PlannerClassRandomStateStaysInBounds) {
  PlannerClass planner(FORWARD, planner_config_);
  double x_min, x_max, y_min, y_max;
  planning_utils::getMapBounds(planner_config_, x_min, x_max, y_min, y_max);

  for (int i = 0; i < 20; ++i) {
    const planning_utils::State s = planner.randomState(planner_config_);
    EXPECT_GE(s.pos.x(), x_min);
    EXPECT_LE(s.pos.x(), x_max);
    EXPECT_GE(s.pos.y(), y_min);
    EXPECT_LE(s.pos.y(), y_max);
    EXPECT_NEAR(s.pos.z(), planner_config_.h_nom, 1e-6);
    EXPECT_LE(s.vel.head<2>().norm(), planner_config_.v_max + 1e-9);
    EXPECT_TRUE(planning_utils::isValidState(s, planner_config_, LEAP_STANCE));
  }
}

TEST_F(GlobalBodyPlannerTestFixture, RrtPathAndSequenceHelpersFollowParents) {
  PlannerClass planner(FORWARD, planner_config_);
  planning_utils::State root = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State mid = makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State end = makeState(2.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planner.init(root);
  planner.addVertex(1, mid);
  planner.addEdge(0, 1, 1.0);
  planner.addAction(1, makeConnectAction());
  planner.addVertex(2, end);
  planner.addEdge(1, 2, 1.0);
  planner.addAction(2, makeConnectAction(2.0));

  RRT rrt;
  const std::vector<int> path = rrt.pathFromStart(planner, 2);
  const std::vector<planning_utils::State> states =
      rrt.getStateSequence(planner, path);
  const std::vector<planning_utils::Action> actions =
      rrt.getActionSequence(planner, path);

  EXPECT_EQ(path, (std::vector<int>{0, 1, 2}));
  ASSERT_EQ(states.size(), 3u);
  EXPECT_TRUE(states.front().isApprox(root));
  EXPECT_TRUE(states.back().isApprox(end));
  ASSERT_EQ(actions.size(), 2u);
  EXPECT_DOUBLE_EQ(actions[0].t_s_leap, 1.0);
  EXPECT_DOUBLE_EQ(actions[1].t_s_leap, 2.0);
}

TEST_F(GlobalBodyPlannerTestFixture, AttemptConnectRejectsTrappedDuration) {
  RRT rrt;
  planning_utils::State s0 = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State s1 = makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::StateActionResult result;

  EXPECT_EQ(rrt.attemptConnect(s0, s1, planner_config_.dt, result,
                               planner_config_, FORWARD),
            TRAPPED);
}

TEST_F(GlobalBodyPlannerTestFixture, ExtendAddsTimedVertexWhenAdvanced) {
  PlannerClass planner(FORWARD, planner_config_);
  planner.init(makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0));
  RRT rrt;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub;

  const int result =
      rrt.extend(planner, makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0),
                 planner_config_, FORWARD, tree_pub);

  EXPECT_NE(result, TRAPPED);
  ASSERT_EQ(planner.getNumVertices(), 2);
  EXPECT_GT(planner.getTime(1), planner.getTime(0));
}

TEST_F(GlobalBodyPlannerTestFixture, GbplFindPlanRejectsInvalidInputs) {
  TestGBPL gbpl;
  std::vector<planning_utils::State> states;
  std::vector<planning_utils::Action> actions;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub;

  EXPECT_EQ(gbpl.findPlan(planner_config_,
                          makeState(20.0, 0.0, 0.3, 0.0, 0.0, 0.0),
                          makeState(1.0, 0.0, 0.3, 0.0, 0.0, 0.0), states,
                          actions, tree_pub),
            INVALID_START_STATE);
  EXPECT_EQ(gbpl.findPlan(planner_config_,
                          makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0),
                          makeState(20.0, 0.0, 0.3, 0.0, 0.0, 0.0), states,
                          actions, tree_pub),
            INVALID_GOAL_STATE);
  EXPECT_EQ(gbpl.findPlan(planner_config_,
                          makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0),
                          makeState(0.01, 0.0, 0.3, 0.0, 0.0, 0.0), states,
                          actions, tree_pub),
            INVALID_START_GOAL_EQUAL);
}

TEST_F(GlobalBodyPlannerTestFixture, GbplWarmStartRetainsCachedTrees) {
  TestGBPL gbpl;
  planner_config_.max_planning_time = 0.0;
  std::vector<planning_utils::State> states;
  std::vector<planning_utils::Action> actions;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub;

  gbpl.setWarmStart(false);
  gbpl.findPlan(planner_config_, makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0),
                makeState(2.0, 0.0, 0.3, 0.0, 0.0, 0.0), states, actions,
                tree_pub);
  ASSERT_TRUE(gbpl.hasCache());
  const PlannerClass* forward_cache = gbpl.forwardCache();
  const PlannerClass* reverse_cache = gbpl.reverseCache();

  planner_config_.dynamic_constraints.push_back(makeConstraint(
      Eigen::Vector3d(0.0, 0.0, 0.3), 0.0, Eigen::Vector3d(0.5, 0.25, 0.2),
      0.0, 10.0));
  gbpl.setWarmStart(true);
  gbpl.findPlan(planner_config_, makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0),
                makeState(2.0, 0.0, 0.3, 0.0, 0.0, 0.0), states, actions,
                tree_pub);

  EXPECT_TRUE(gbpl.hasCache());
  EXPECT_EQ(gbpl.forwardCache(), forward_cache);
  EXPECT_EQ(gbpl.reverseCache(), reverse_cache);
}

TEST_F(GlobalBodyPlannerTestFixture, GbplExtractClosestPathReturnsTreePrefix) {
  PlannerClass planner(FORWARD, planner_config_);
  planning_utils::State root = makeState(0.0, 0.0, 0.3, 0.0, 0.0, 0.0);
  planning_utils::State child = makeState(0.5, 0.0, 0.3, 0.0, 0.0, 0.0);
  planner.init(root);
  planner.addVertex(1, child);
  planner.addEdge(0, 1, 0.5);
  planner.addAction(1, makeConnectAction(1.0));

  TestGBPL gbpl;
  std::vector<planning_utils::State> states;
  std::vector<planning_utils::Action> actions;
  gbpl.extractClosestPath(planner, makeState(0.6, 0.0, 0.3, 0.0, 0.0, 0.0),
                          states, actions, planner_config_);

  ASSERT_GE(states.size(), 2u);
  EXPECT_TRUE(states.front().isApprox(root));
  EXPECT_TRUE(states.back().isApprox(child));
  EXPECT_EQ(actions.size(), states.size() - 1);
}
