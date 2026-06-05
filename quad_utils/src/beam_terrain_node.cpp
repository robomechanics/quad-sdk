#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "quad_utils/beam_terrain_converter.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("beam_terrain");
  auto converter = std::make_shared<beam_terrain::BeamTerrainConverter>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
