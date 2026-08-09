#include "quad_perception/unitree_pointcloud_conversion.hpp"

namespace quad_perception {

void convertDdsToRos(const sensor_msgs::msg::dds_::PointCloud2_& dds_msg,
                     sensor_msgs::msg::PointCloud2& ros_msg,
                     const std::string& frame_id_override) {
  ros_msg.header.stamp.sec = dds_msg.header().stamp().sec();
  ros_msg.header.stamp.nanosec = dds_msg.header().stamp().nanosec();
  ros_msg.header.frame_id =
      frame_id_override.empty() ? dds_msg.header().frame_id() : frame_id_override;

  ros_msg.height = dds_msg.height();
  ros_msg.width = dds_msg.width();
  ros_msg.is_bigendian = dds_msg.is_bigendian();
  ros_msg.point_step = dds_msg.point_step();
  ros_msg.row_step = dds_msg.row_step();
  ros_msg.is_dense = dds_msg.is_dense();

  const auto& dds_fields = dds_msg.fields();
  ros_msg.fields.clear();
  ros_msg.fields.reserve(dds_fields.size());
  for (const auto& f : dds_fields) {
    sensor_msgs::msg::PointField ros_field;
    ros_field.name = f.name();
    ros_field.offset = f.offset();
    ros_field.datatype = f.datatype();
    ros_field.count = f.count();
    ros_msg.fields.push_back(std::move(ros_field));
  }

  ros_msg.data = dds_msg.data();
}

}  // namespace quad_perception
