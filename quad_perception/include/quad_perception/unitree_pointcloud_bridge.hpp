#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace quad_perception {

// Bridges Unitree's DDS point-cloud topic (default: rt/utlidar/cloud_deskewed) to
// a ROS 2 sensor_msgs/msg/PointCloud2 publisher. Runs on-robot; requires the Go2
// LiDAR to be active and the Unitree SDK2 network interface reachable.
class UnitreePointCloudBridge : public rclcpp::Node {
 public:
  UnitreePointCloudBridge();
  ~UnitreePointCloudBridge() override;

  UnitreePointCloudBridge(const UnitreePointCloudBridge&) = delete;
  UnitreePointCloudBridge& operator=(const UnitreePointCloudBridge&) = delete;

 private:
  void ddsCallback(const void* message);

  std::string input_dds_topic_;
  std::string frame_id_override_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<
      sensor_msgs::msg::dds_::PointCloud2_>> sub_;
};

}  // namespace quad_perception
