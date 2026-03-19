#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "quad_utils/filters_demo.hpp"  // adjust if your header path differs

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<grid_map_demos::FiltersDemo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
