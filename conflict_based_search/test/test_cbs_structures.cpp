#include <gtest/gtest.h>

#include <memory>
#include <queue>
#include <vector>

#include "conflict_based_search/conflict_based_search.hpp"

TEST(ConflictBasedSearchStructuresTest, CBSNodeUpdateCostSumsKnownRobots) {
  conflict_based_search::CBSNode node;
  node.robot_names = {"robot_1", "robot_2", "robot_3"};
  node.cost_map["robot_1"] = 1.5;
  node.cost_map["robot_2"] = 2.0;

  node.updateCost();

  EXPECT_DOUBLE_EQ(node.cost, 3.5);
}

TEST(ConflictBasedSearchStructuresTest, CBSNodeCompareCreatesMinHeap) {
  auto low = std::make_shared<conflict_based_search::CBSNode>();
  auto high = std::make_shared<conflict_based_search::CBSNode>();
  low->cost = 1.0;
  high->cost = 5.0;

  std::priority_queue<std::shared_ptr<conflict_based_search::CBSNode>,
                      std::vector<std::shared_ptr<conflict_based_search::CBSNode>>,
                      conflict_based_search::CBSNodeCompare>
      open;
  open.push(high);
  open.push(low);

  EXPECT_EQ(open.top(), low);
}
