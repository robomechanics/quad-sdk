#include "quad_perception/pointcloud_to_gridmap.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_conversions/pcl_conversions.h>
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

  ground_segmentation_ = this->declare_parameter<bool>("ground_segmentation", true);
  outlier_mean_k_ = this->declare_parameter<int>("outlier_mean_k", 10);
  outlier_stddev_ = this->declare_parameter<double>("outlier_stddev", 1.0);
  ground_cell_size_ = this->declare_parameter<double>("ground_cell_size", 0.3);
  ground_max_window_size_ =
      this->declare_parameter<int>("ground_max_window_size", 8);
  ground_slope_ = this->declare_parameter<double>("ground_slope", 0.5);
  ground_initial_distance_ =
      this->declare_parameter<double>("ground_initial_distance", 0.15);
  ground_max_distance_ =
      this->declare_parameter<double>("ground_max_distance", 1.0);

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

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud, *raw);

  // Height-clip before ground fitting so far-off returns (ceiling, multipath)
  // don't skew the morphological filter's notion of "low".
  pcl::PointCloud<pcl::PointXYZ>::Ptr clipped(new pcl::PointCloud<pcl::PointXYZ>);
  clipped->reserve(raw->size());
  for (const auto& p : *raw) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        p.z >= min_z_ && p.z <= max_z_) {
      clipped->push_back(p);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground = clipped;

  if (ground_segmentation_ && !clipped->empty()) {
    // Drop sparse noise (reflections, single stray returns) before ground
    // fitting; PMF otherwise treats isolated high outliers as terrain.
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(clipped);
    sor.setMeanK(outlier_mean_k_);
    sor.setStddevMulThresh(outlier_stddev_);
    sor.filter(*denoised);

    // Separates the walkable surface from obstacles (furniture, overhangs)
    // sitting above it, so those don't get rasterized as terrain steps.
    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(denoised);
    pmf.setCellSize(static_cast<float>(ground_cell_size_));
    pmf.setMaxWindowSize(ground_max_window_size_);
    pmf.setSlope(static_cast<float>(ground_slope_));
    pmf.setInitialDistance(static_cast<float>(ground_initial_distance_));
    pmf.setMaxDistance(static_cast<float>(ground_max_distance_));
    pmf.extract(ground_indices->indices);

    ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(denoised);
    extract.setIndices(ground_indices);
    extract.filter(*ground);
  }

  for (const auto& p : *ground) {
    const grid_map::Position position(p.x, p.y);
    if (!map_.isInside(position)) {
      continue;
    }
    float& cell = map_.atPosition(layer_, position);
    cell = std::isnan(cell) ? p.z : std::max(cell, p.z);
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
