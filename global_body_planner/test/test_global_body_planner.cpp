#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "global_body_planner/global_body_planner.hpp"
#include "test_helpers.hpp"

TEST(GlobalBodyPlannerNodeTest, ConstructsWithYamlConfiguration) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));

  EXPECT_NO_THROW({
    GlobalBodyPlanner global_body_planner(node);
  });
}
