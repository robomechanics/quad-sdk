#pragma once

#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

namespace quad_perception {

// Translate a Unitree DDS PointCloud2_ into a ROS 2 sensor_msgs::msg::PointCloud2.
// Copies field metadata and the raw byte blob one-for-one; the two IDLs share the
// same wire format (float32 x/y/z/intensity, little-endian on the Go2).
//
// If frame_id_override is non-empty, it replaces the DDS message's frame_id in
// the ROS output; otherwise the DDS frame_id passes through.
void convertDdsToRos(const sensor_msgs::msg::dds_::PointCloud2_& dds_msg,
                     sensor_msgs::msg::PointCloud2& ros_msg,
                     const std::string& frame_id_override = "");

}  // namespace quad_perception
