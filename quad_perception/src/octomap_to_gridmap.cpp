#include "quad_perception/octomap_to_gridmap.hpp"

#include <memory>

#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

namespace quad_perception {

OctomapToGridMap::OctomapToGridMap() : rclcpp::Node("octomap_to_gridmap") {
  layer_ = this->declare_parameter<std::string>("layer", "z");
  ground_min_z_ = this->declare_parameter<double>("ground_min_z", -0.5);
  ground_max_z_ = this->declare_parameter<double>("ground_max_z", 0.5);
  const std::string input_topic =
      this->declare_parameter<std::string>("input_topic", "/octomap_full");
  const std::string output_topic = this->declare_parameter<std::string>(
      "output_topic", "/mapping/terrain_map_raw");

  // Matches filter_chain.yaml's input_topic QoS.
  const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic, qos);
  sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      input_topic, rclcpp::QoS(1).reliable(),
      std::bind(&OctomapToGridMap::octomapCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "Projecting octomap '%s' -> grid_map layer '%s' on '%s', "
              "ground band [%.2f, %.2f] m",
              input_topic.c_str(), layer_.c_str(), output_topic.c_str(),
              ground_min_z_, ground_max_z_);
}

void OctomapToGridMap::octomapCallback(
    const octomap_msgs::msg::Octomap::SharedPtr msg) {
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
      octomap_msgs::fullMsgToMap(*msg));
  if (!abstract_tree) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to deserialize incoming octomap message");
    return;
  }
  auto* tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "Incoming octomap is not an OcTree");
    return;
  }

  double min_x, min_y, min_tree_z, max_x, max_y, max_tree_z;
  tree->getMetricMin(min_x, min_y, min_tree_z);
  tree->getMetricMax(max_x, max_y, max_tree_z);
  // XY bounds come from the tree's own extent; Z is clamped to the ground
  // band so anything above it (furniture, overhangs) is excluded from the
  // terrain layer rather than read as a step.
  const grid_map::Position3 min_point(min_x, min_y, ground_min_z_);
  const grid_map::Position3 max_point(max_x, max_y, ground_max_z_);

  grid_map::GridMap grid_map;
  if (!grid_map::GridMapOctomapConverter::fromOctomap(
          *tree, layer_, grid_map, &min_point, &max_point)) {
    RCLCPP_WARN(this->get_logger(), "Failed to convert octomap to grid map");
    return;
  }

  grid_map.setFrameId(msg->header.frame_id);
  grid_map.setTimestamp(this->now().nanoseconds());
  auto out_msg = grid_map::GridMapRosConverter::toMessage(grid_map);
  pub_->publish(std::move(*out_msg));
}

}  // namespace quad_perception
