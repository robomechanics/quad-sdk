#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include "quad_perception/unitree_pointcloud_conversion.hpp"

namespace {

// Build a synthetic Unitree DDS PointCloud2_ matching the shape Unitree publishes
// on rt/utlidar/cloud_deskewed: unordered (height=1), 4 float32 fields
// (x, y, z, intensity), little-endian.
sensor_msgs::msg::dds_::PointCloud2_ makeSyntheticDdsCloud(
    const std::vector<std::array<float, 4>>& points,
    const std::string& frame_id, int32_t sec, uint32_t nanosec) {
  sensor_msgs::msg::dds_::PointCloud2_ msg;
  msg.header().stamp().sec() = sec;
  msg.header().stamp().nanosec() = nanosec;
  msg.header().frame_id() = frame_id;

  msg.height() = 1;
  msg.width() = static_cast<uint32_t>(points.size());
  msg.is_bigendian() = false;
  msg.is_dense() = true;

  constexpr uint8_t FLOAT32 = 7;  // sensor_msgs::msg::PointField::FLOAT32
  const char* names[4] = {"x", "y", "z", "intensity"};
  msg.fields().resize(4);
  for (uint32_t i = 0; i < 4; ++i) {
    msg.fields()[i].name() = names[i];
    msg.fields()[i].offset() = i * sizeof(float);
    msg.fields()[i].datatype() = FLOAT32;
    msg.fields()[i].count() = 1;
  }
  msg.point_step() = 4 * sizeof(float);
  msg.row_step() = msg.point_step() * msg.width();

  msg.data().resize(msg.row_step());
  for (size_t i = 0; i < points.size(); ++i) {
    std::memcpy(msg.data().data() + i * msg.point_step(), points[i].data(),
                msg.point_step());
  }
  return msg;
}

// Read the 4 float fields at row i out of a ROS PointCloud2 blob.
std::array<float, 4> readPoint(const sensor_msgs::msg::PointCloud2& msg,
                               size_t i) {
  std::array<float, 4> pt{};
  std::memcpy(pt.data(), msg.data.data() + i * msg.point_step,
              msg.point_step);
  return pt;
}

}  // namespace

TEST(UnitreePointCloudConversionTest, RoundTripsHeaderAndFields) {
  const std::vector<std::array<float, 4>> points = {
      {1.0f, 2.0f, 3.0f, 42.0f},
      {-0.5f, 0.25f, -1.25f, 7.0f},
      {100.0f, -50.0f, 0.0f, 0.0f},
  };
  const auto dds_msg = makeSyntheticDdsCloud(points, "utlidar_lidar",
                                             /*sec=*/1704067200,
                                             /*nanosec=*/123456789u);

  sensor_msgs::msg::PointCloud2 ros_msg;
  quad_perception::convertDdsToRos(dds_msg, ros_msg);

  EXPECT_EQ(ros_msg.header.stamp.sec, 1704067200);
  EXPECT_EQ(ros_msg.header.stamp.nanosec, 123456789u);
  EXPECT_EQ(ros_msg.header.frame_id, "utlidar_lidar");

  EXPECT_EQ(ros_msg.height, 1u);
  EXPECT_EQ(ros_msg.width, points.size());
  EXPECT_EQ(ros_msg.is_bigendian, false);
  EXPECT_EQ(ros_msg.is_dense, true);
  EXPECT_EQ(ros_msg.point_step, 4u * sizeof(float));
  EXPECT_EQ(ros_msg.row_step, ros_msg.point_step * ros_msg.width);

  ASSERT_EQ(ros_msg.fields.size(), 4u);
  const std::array<const char*, 4> expected_names = {"x", "y", "z", "intensity"};
  for (uint32_t i = 0; i < 4; ++i) {
    EXPECT_EQ(ros_msg.fields[i].name, expected_names[i]);
    EXPECT_EQ(ros_msg.fields[i].offset, i * sizeof(float));
    EXPECT_EQ(ros_msg.fields[i].datatype, 7u);  // FLOAT32
    EXPECT_EQ(ros_msg.fields[i].count, 1u);
  }
}

TEST(UnitreePointCloudConversionTest, PreservesPointBytesExactly) {
  const std::vector<std::array<float, 4>> points = {
      {1.0f, 2.0f, 3.0f, 42.0f},
      {-0.5f, 0.25f, -1.25f, 7.0f},
      {100.0f, -50.0f, 0.0f, 0.0f},
  };
  const auto dds_msg = makeSyntheticDdsCloud(points, "utlidar_lidar", 0, 0);

  sensor_msgs::msg::PointCloud2 ros_msg;
  quad_perception::convertDdsToRos(dds_msg, ros_msg);

  ASSERT_EQ(ros_msg.data.size(), points.size() * ros_msg.point_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto pt = readPoint(ros_msg, i);
    EXPECT_FLOAT_EQ(pt[0], points[i][0]);
    EXPECT_FLOAT_EQ(pt[1], points[i][1]);
    EXPECT_FLOAT_EQ(pt[2], points[i][2]);
    EXPECT_FLOAT_EQ(pt[3], points[i][3]);
  }
}

TEST(UnitreePointCloudConversionTest, FrameIdOverrideReplacesFrame) {
  const auto dds_msg = makeSyntheticDdsCloud({{0.0f, 0.0f, 0.0f, 0.0f}},
                                             "utlidar_lidar", 0, 0);

  sensor_msgs::msg::PointCloud2 ros_msg;
  quad_perception::convertDdsToRos(dds_msg, ros_msg, /*frame_id_override=*/"odom");

  EXPECT_EQ(ros_msg.header.frame_id, "odom");
}

TEST(UnitreePointCloudConversionTest, EmptyCloudProducesEmptyRosMessage) {
  const auto dds_msg = makeSyntheticDdsCloud({}, "utlidar_lidar", 0, 0);

  sensor_msgs::msg::PointCloud2 ros_msg;
  quad_perception::convertDdsToRos(dds_msg, ros_msg);

  EXPECT_EQ(ros_msg.width, 0u);
  EXPECT_EQ(ros_msg.row_step, 0u);
  EXPECT_TRUE(ros_msg.data.empty());
  EXPECT_EQ(ros_msg.fields.size(), 4u);  // schema still present
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
