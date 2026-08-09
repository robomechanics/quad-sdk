#ifndef QUAD_PERCEPTION_POINTCLOUD_TO_GRIDMAP_HPP_
#define QUAD_PERCEPTION_POINTCLOUD_TO_GRIDMAP_HPP_

#include <memory>
#include <string>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace quad_perception {

/**
 * @brief Rasterizes LiDAR point clouds into a grid_map elevation layer.
 *
 * Publishes the same message the rest of quad-sdk's terrain pipeline expects:
 * a grid_map_msgs/GridMap carrying a 'z' layer on terrain_map_raw, latched
 * (transient local), which the existing grid_map filter chain turns into the
 * inpainted/normals/traversability terrain_map.
 *
 * Works with or without a TF tree. With map_frame empty the grid is built in
 * the cloud's own frame, which is enough to visualize a LiDAR on its own; set
 * map_frame once a TF to a fixed frame exists and the grid accumulates there
 * instead, following the sensor.
 */
class PointCloudToGridMap : public rclcpp::Node {
 public:
  PointCloudToGridMap();

 private:
  /// Rasterize one cloud into the map, transforming into map_frame_ if set.
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// Publish the current map; decoupled from the cloud rate.
  void publishMap();

  /// Fold one cloud's points into the elevation layer.
  void rasterize(const sensor_msgs::msg::PointCloud2& cloud);

  grid_map::GridMap map_;
  std::string layer_;
  std::string map_frame_;
  double min_z_ = -2.0;
  double max_z_ = 2.0;
  bool accumulate_ = true;
  bool follow_sensor_ = true;
  bool have_geometry_ = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace quad_perception

#endif  // QUAD_PERCEPTION_POINTCLOUD_TO_GRIDMAP_HPP_
