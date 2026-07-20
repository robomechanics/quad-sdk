#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "global_body_planner/global_body_planner.hpp"
#include "global_body_planner/global_body_planner_test_fixture.hpp"

TEST(GlobalBodyPlannerNodeTest, ConstructsWithYamlConfiguration) {
  auto node = std::make_shared<rclcpp::Node>(
      "global_body_planner",
      global_body_planner_test::plannerNodeOptions(true));

  EXPECT_NO_THROW({
    GlobalBodyPlanner global_body_planner(node);
  });
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
