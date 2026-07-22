#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "quad_utils/rviz_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <quad_msgs/msg/multi_foot_plan_continuous.hpp>
#include <quad_msgs/msg/multi_foot_plan_discrete.hpp>
#include <quad_msgs/msg/robot_plan.hpp>
#include <quad_msgs/msg/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {
constexpr double kTol = 1e-9;

rclcpp::NodeOptions rvizOptions() {
  std::string pkg_share =
      ament_index_cpp::get_package_share_directory("quad_utils");
  std::string params_file = pkg_share + "/config/rviz_visualization.yaml";

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_file});
  options.append_parameter_override("tf_prefix", "robot_1");
  options.append_parameter_override("use_sim_time", false);
  return options;
}

template <typename MsgT>
bool spinUntilMessage(const rclcpp::Node::SharedPtr& node_a,
                      const rclcpp::Node::SharedPtr& node_b,
                      const std::shared_ptr<MsgT>& msg,
                      const std::chrono::milliseconds timeout =
                          std::chrono::milliseconds(750)) {
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !msg &&
         std::chrono::steady_clock::now() - start < timeout) {
    rclcpp::spin_some(node_a);
    rclcpp::spin_some(node_b);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return msg != nullptr;
}

quad_msgs::msg::RobotState makeRobotState(double x, double y, double z) {
  quad_msgs::msg::RobotState state;
  state.header.frame_id = "map";
  state.body.pose.position.x = x;
  state.body.pose.position.y = y;
  state.body.pose.position.z = z;
  state.body.pose.orientation.w = 1.0;
  state.joints.name = {"8",  "0", "1", "9",  "2", "3",
                       "10", "4", "5", "11", "6", "7"};
  state.joints.position.resize(12, 0.0);
  state.joints.velocity.resize(12, 0.0);
  state.joints.effort.resize(12, 0.0);
  return state;
}

quad_msgs::msg::GRFArray makeGrfArray() {
  quad_msgs::msg::GRFArray grfs;
  grfs.header.frame_id = "map";
  grfs.points.resize(1);
  grfs.vectors.resize(1);
  grfs.contact_states.resize(1, true);
  grfs.points[0].z = 0.1;
  grfs.vectors[0].z = 100.0;
  return grfs;
}

quad_msgs::msg::RobotPlan makeRobotPlan() {
  quad_msgs::msg::RobotPlan plan;
  plan.header.frame_id = "map";
  plan.states.push_back(makeRobotState(0.0, 0.0, 0.3));
  plan.states.push_back(makeRobotState(0.5, 0.0, 0.3));
  plan.grfs.push_back(makeGrfArray());
  plan.grfs.push_back(makeGrfArray());
  plan.primitive_ids = {0, 2};
  return plan;
}

}  // namespace

TEST(RVizInterface, PublishesPlanMarkersFromInputPlans) {
  auto rviz_node =
      std::make_shared<rclcpp::Node>("rviz_interface_plan_test", rvizOptions());
  RVizInterface rviz_interface(rviz_node);
  auto test_node = std::make_shared<rclcpp::Node>("rviz_plan_test_driver");

  std::shared_ptr<visualization_msgs::msg::Marker> global_marker;
  std::shared_ptr<visualization_msgs::msg::MarkerArray> global_grfs;
  auto global_marker_sub =
      test_node->create_subscription<visualization_msgs::msg::Marker>(
          "visualization/global_plan", 1,
          [&](visualization_msgs::msg::Marker::SharedPtr msg) {
            global_marker = msg;
          });
  auto global_grf_sub =
      test_node->create_subscription<visualization_msgs::msg::MarkerArray>(
          "visualization/global_plan_grf", 1,
          [&](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            global_grfs = msg;
          });
  auto global_plan_pub =
      test_node->create_publisher<quad_msgs::msg::RobotPlan>("global_plan", 1);

  const auto plan = makeRobotPlan();
  for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
    global_plan_pub->publish(plan);
    if (spinUntilMessage(rviz_node, test_node, global_marker) &&
        spinUntilMessage(rviz_node, test_node, global_grfs)) {
      break;
    }
  }

  ASSERT_NE(global_marker, nullptr);
  ASSERT_NE(global_grfs, nullptr);
  EXPECT_EQ(global_marker->type, visualization_msgs::msg::Marker::LINE_STRIP);
  EXPECT_EQ(global_marker->points.size(), 2u);
  EXPECT_EQ(global_grfs->markers.size(), 2u);
}

TEST(RVizInterface, PublishesCurrentGrfMarkers) {
  auto rviz_node =
      std::make_shared<rclcpp::Node>("rviz_interface_grf_test", rvizOptions());
  RVizInterface rviz_interface(rviz_node);
  auto test_node = std::make_shared<rclcpp::Node>("rviz_grf_test_driver");

  std::shared_ptr<visualization_msgs::msg::MarkerArray> current_grfs;
  auto current_grf_sub =
      test_node->create_subscription<visualization_msgs::msg::MarkerArray>(
          "visualization/current_grf", 1,
          [&](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            current_grfs = msg;
          });
  auto grf_pub =
      test_node->create_publisher<quad_msgs::msg::GRFArray>("control/grfs", 1);

  quad_msgs::msg::GRFArray grfs;
  grfs.header.frame_id = "map";
  grfs.points.resize(2);
  grfs.vectors.resize(2);
  grfs.contact_states = {true, false};
  grfs.points[0].x = 1.0;
  grfs.points[0].z = 0.2;
  grfs.vectors[0].z = 100.0;
  grfs.points[1].x = 2.0;
  grfs.points[1].z = 0.2;
  grfs.vectors[1].z = 50.0;

  for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
    grf_pub->publish(grfs);
    if (spinUntilMessage(rviz_node, test_node, current_grfs)) {
      break;
    }
  }

  ASSERT_NE(current_grfs, nullptr);
  ASSERT_EQ(current_grfs->markers.size(), 2u);
  EXPECT_EQ(current_grfs->markers[0].type,
            visualization_msgs::msg::Marker::ARROW);
  ASSERT_EQ(current_grfs->markers[0].points.size(), 2u);
  EXPECT_NEAR(current_grfs->markers[0].points[1].z, 0.4, kTol);
  EXPECT_NEAR(current_grfs->markers[0].color.a, 1.0, kTol);
  EXPECT_NEAR(current_grfs->markers[1].color.a, 0.0, kTol);
}

TEST(RVizInterface, PublishesFootAndStateVisualization) {
  auto rviz_node =
      std::make_shared<rclcpp::Node>("rviz_interface_state_test", rvizOptions());
  RVizInterface rviz_interface(rviz_node);
  auto test_node = std::make_shared<rclcpp::Node>("rviz_state_test_driver");

  std::shared_ptr<visualization_msgs::msg::Marker> foot_marker;
  std::shared_ptr<nav_msgs::msg::Path> foot_path;
  std::shared_ptr<sensor_msgs::msg::JointState> joint_state;
  auto foot_discrete_sub =
      test_node->create_subscription<visualization_msgs::msg::Marker>(
          "visualization/foot_plan_discrete", 1,
          [&](visualization_msgs::msg::Marker::SharedPtr msg) {
            foot_marker = msg;
          });
  auto foot_path_sub = test_node->create_subscription<nav_msgs::msg::Path>(
      "visualization/foot_0_plan_continuous", 1,
      [&](nav_msgs::msg::Path::SharedPtr msg) { foot_path = msg; });
  auto joint_state_sub =
      test_node->create_subscription<sensor_msgs::msg::JointState>(
          "estimate/visualization/joint_states", 1,
          [&](sensor_msgs::msg::JointState::SharedPtr msg) {
            joint_state = msg;
          });

  auto foot_discrete_pub =
      test_node->create_publisher<quad_msgs::msg::MultiFootPlanDiscrete>(
          "foot_plan_discrete", 1);
  auto foot_continuous_pub =
      test_node->create_publisher<quad_msgs::msg::MultiFootPlanContinuous>(
          "foot_plan_continuous", 1);
  auto state_pub = test_node->create_publisher<quad_msgs::msg::RobotState>(
      "state/estimate", 1);

  quad_msgs::msg::MultiFootPlanDiscrete discrete_feet;
  discrete_feet.header.frame_id = "map";
  discrete_feet.feet.resize(4);
  for (auto& foot : discrete_feet.feet) {
    quad_msgs::msg::FootState foothold;
    foothold.position.z = 0.1;
    foot.footholds.push_back(foothold);
  }

  quad_msgs::msg::MultiFootPlanContinuous continuous_feet;
  continuous_feet.header.frame_id = "map";
  continuous_feet.states.resize(2);
  for (auto& state : continuous_feet.states) {
    state.feet.resize(4);
  }

  const auto state = makeRobotState(0.0, 0.0, 0.3);
  for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
    foot_discrete_pub->publish(discrete_feet);
    foot_continuous_pub->publish(continuous_feet);
    state_pub->publish(state);
    if (spinUntilMessage(rviz_node, test_node, foot_marker) &&
        spinUntilMessage(rviz_node, test_node, foot_path) &&
        spinUntilMessage(rviz_node, test_node, joint_state)) {
      break;
    }
  }

  ASSERT_NE(foot_marker, nullptr);
  ASSERT_NE(foot_path, nullptr);
  ASSERT_NE(joint_state, nullptr);
  EXPECT_EQ(foot_marker->points.size(), 4u);
  EXPECT_EQ(foot_path->poses.size(), 2u);
  EXPECT_EQ(joint_state->name.size(), 12u);
}
