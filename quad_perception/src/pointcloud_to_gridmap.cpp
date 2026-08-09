#include "quad_perception/pointcloud_to_gridmap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace quad_perception {

PointCloudToGridMap::PointCloudToGridMap()
    : rclcpp::Node("pointcloud_to_gridmap") {
  const std::string input_topic =
      this->declare_parameter<std::string>("input_topic", "cloud_deskewed");
  // Matches filter_chain.yaml's input_topic and the Isaac flat terrain
  // publisher. Note topics_global.yaml names a bare /terrain_map_raw instead,
  // but nothing subscribes to that one.
  const std::string output_topic = this->declare_parameter<std::string>(
      "output_topic", "/mapping/terrain_map_raw");
  layer_ = this->declare_parameter<std::string>("layer", "z");
  // Empty means "build the grid in whatever frame the cloud arrives in", so
  // this runs standalone before any TF tree exists.
  map_frame_ = this->declare_parameter<std::string>("map_frame", "");
  const double resolution = this->declare_parameter<double>("resolution", 0.05);
  const double length_x = this->declare_parameter<double>("length_x", 10.0);
  const double length_y = this->declare_parameter<double>("length_y", 10.0);
  min_z_ = this->declare_parameter<double>("min_z", -2.0);
  max_z_ = this->declare_parameter<double>("max_z", 2.0);
  accumulate_ = this->declare_parameter<bool>("accumulate", true);
  follow_sensor_ = this->declare_parameter<bool>("follow_sensor", true);
  const double publish_rate = this->declare_parameter<double>("publish_rate", 5.0);

  map_ = grid_map::GridMap({layer_});
  map_.setGeometry(grid_map::Length(length_x, length_y), resolution,
                   grid_map::Position(0.0, 0.0));
  map_[layer_].setConstant(NAN);

  if (!map_frame_.empty()) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Latched: the filter chain and local_planner both subscribe transient_local,
  // so a late subscriber still gets the current map instead of waiting a cycle.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic, qos);
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudToGridMap::cloudCallback, this,
                std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_rate, 0.1)),
      std::bind(&PointCloudToGridMap::publishMap, this));

  RCLCPP_INFO(this->get_logger(),
              "Rasterizing '%s' -> '%s' at %.3f m over %.1fx%.1f m, frame '%s'",
              input_topic.c_str(), output_topic.c_str(), resolution, length_x,
              length_y, map_frame_.empty() ? "<cloud frame>" : map_frame_.c_str());
}

void PointCloudToGridMap::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (map_frame_.empty()) {
    // No TF needed: the cloud frame is the map frame.
    if (!have_geometry_) {
      map_.setFrameId(msg->header.frame_id);
      have_geometry_ = true;
    }
    rasterize(*msg);
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
                                     msg->header.stamp,
                                     tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No transform %s <- %s yet: %s", map_frame_.c_str(),
                         msg->header.frame_id.c_str(), ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 transformed;
  tf2::doTransform(*msg, transformed, tf);

  if (!have_geometry_) {
    map_.setFrameId(map_frame_);
    have_geometry_ = true;
  }
  if (follow_sensor_) {
    // Keep the sensor inside the map; move() shifts the grid and blanks only
    // the cells newly brought into view, preserving everything else.
    map_.move(grid_map::Position(tf.transform.translation.x,
                                 tf.transform.translation.y));
  }
  rasterize(transformed);
}

void PointCloudToGridMap::rasterize(const sensor_msgs::msg::PointCloud2& cloud) {
  if (!accumulate_) {
    map_[layer_].setConstant(NAN);
  }

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float x = *it_x, y = *it_y, z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (z < min_z_ || z > max_z_) {
      continue;
    }
    const grid_map::Position position(x, y);
    if (!map_.isInside(position)) {
      continue;
    }
    float& cell = map_.atPosition(layer_, position);
    // Highest return wins, so obstacles survive rather than being averaged
    // away against the ground beneath them.
    cell = std::isnan(cell) ? z : std::max(cell, z);
  }
}

void PointCloudToGridMap::publishMap() {
  if (!have_geometry_) {
    return;
  }
  map_.setTimestamp(this->now().nanoseconds());
  auto msg = grid_map::GridMapRosConverter::toMessage(map_);
  pub_->publish(std::move(*msg));
}

}  // namespace quad_perception
