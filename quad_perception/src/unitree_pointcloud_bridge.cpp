#include "quad_perception/unitree_pointcloud_bridge.hpp"

#include <cstdlib>
#include <functional>

#include <unitree/robot/channel/channel_factory.hpp>

#include "quad_perception/unitree_pointcloud_conversion.hpp"

namespace quad_perception {

namespace {

std::string resolveNetworkInterface(const std::string& param_value) {
  if (!param_value.empty()) {
    return param_value;
  }
  // Match UnitreeInterface convention: env var set by init_robot.sh, fallback eth0.
  const char* env_iface = std::getenv("ROBOT_MCU_IFACE");
  if (env_iface && *env_iface) {
    return std::string(env_iface);
  }
  return "eth0";
}

}  // namespace

UnitreePointCloudBridge::UnitreePointCloudBridge()
    : rclcpp::Node("unitree_pointcloud_bridge") {
  const std::string network_interface = resolveNetworkInterface(
      this->declare_parameter<std::string>("network_interface", ""));
  input_dds_topic_ = this->declare_parameter<std::string>(
      "input_dds_topic", "rt/utlidar/cloud_deskewed");
  const std::string output_topic = this->declare_parameter<std::string>(
      "output_topic", "cloud_deskewed");
  frame_id_override_ = this->declare_parameter<std::string>("frame_id", "");

  RCLCPP_INFO(this->get_logger(),
              "Initializing Unitree DDS on interface '%s', DDS topic '%s' -> ROS "
              "topic '%s'",
              network_interface.c_str(), input_dds_topic_.c_str(),
              output_topic.c_str());

  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());

  sub_ = std::make_unique<unitree::robot::ChannelSubscriber<
      sensor_msgs::msg::dds_::PointCloud2_>>(input_dds_topic_);
  sub_->InitChannel(
      std::bind(&UnitreePointCloudBridge::ddsCallback, this,
                std::placeholders::_1),
      1);
}

UnitreePointCloudBridge::~UnitreePointCloudBridge() {
  // Ensure the DDS subscriber is torn down before the publisher is destroyed
  // so no callback can fire against a dangling pub_.
  sub_.reset();
}

void UnitreePointCloudBridge::ddsCallback(const void* message) {
  if (!message || !pub_) {
    return;
  }
  const auto* dds_msg =
      static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(message);

  auto ros_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  convertDdsToRos(*dds_msg, *ros_msg, frame_id_override_);
  pub_->publish(std::move(ros_msg));
}

}  // namespace quad_perception
