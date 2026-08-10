#ifndef QUAD_PERCEPTION_OCTOMAP_TO_GRIDMAP_HPP_
#define QUAD_PERCEPTION_OCTOMAP_TO_GRIDMAP_HPP_

#include <string>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace quad_perception {

/*!
 * Projects octomap_server's accumulated 3D occupancy map down into a
 * grid_map elevation layer, restricted to a ground-height band so obstacles
 * that sit above the walkable surface (furniture, overhangs) don't get read
 * as terrain. octomap_server does the hard part upstream of this node:
 * probabilistic log-odds voxel fusion and sensor-origin ray-traced
 * free-space clearing, so transient noise self-corrects instead of
 * accumulating as permanent spikes the way a naive max-height rasterizer
 * would.
 */
class OctomapToGridMap : public rclcpp::Node {
 public:
  OctomapToGridMap();

 private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  std::string layer_;
  double ground_min_z_ = -0.5;
  double ground_max_z_ = 0.5;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
};

}  // namespace quad_perception

#endif  // QUAD_PERCEPTION_OCTOMAP_TO_GRIDMAP_HPP_
