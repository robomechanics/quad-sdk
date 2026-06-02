#include <rclcpp/rclcpp.hpp>
#include "quad_utils/mjcf_to_grid_map_converter.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create the node
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<rclcpp::Node>("mjcf_to_grid_map");

  // Creating converter with a node pointer
  auto mesh_to_grid_map_converter =
      std::make_shared<mjcf_to_grid_map::MjcfToGridMapConverter>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
