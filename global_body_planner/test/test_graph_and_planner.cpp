#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "global_body_planner/global_body_planner_test_fixture.hpp"
#include "test_helpers.hpp"

using global_body_planner_test::makeConstraint;
using global_body_planner_test::makeState;

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
