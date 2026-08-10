#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "quad_perception/octomap_to_gridmap.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quad_perception::OctomapToGridMap>());
  rclcpp::shutdown();
  return 0;
}
