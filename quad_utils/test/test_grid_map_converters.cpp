#include <gtest/gtest.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>

#define private public
#include "quad_utils/mesh_to_grid_map_converter.hpp"
#include "quad_utils/mjcf_to_grid_map_converter.hpp"
#undef private

#include <cstdlib>
#include <chrono>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

std::string sourceRoot() {
  const char* source_dir = std::getenv("QUAD_UTILS_SOURCE_DIR");
  if (source_dir == nullptr) {
    throw std::runtime_error("Missing QUAD_UTILS_SOURCE_DIR");
  }
  return source_dir;
}

std::string flatMeshPath() {
  return sourceRoot() +
         "/quad_simulator/quad_sim_scripts/models/flat/meshes/flat.ply";
}

rclcpp::NodeOptions converterOptions() {
  rclcpp::NodeOptions options;
  options.append_parameter_override("world", "__missing_test_world__");
  options.append_parameter_override("verbose", false);
  options.append_parameter_override("grid_map_resolution", 0.2);
  options.append_parameter_override("layer_name", "elevation");
  options.append_parameter_override("latch_grid_map_pub", true);
  options.append_parameter_override("frame_id_mesh_loaded", "map");
  options.append_parameter_override("use_sim_time", false);
  return options;
}

template <typename ConverterT>
void expectProcessFileService(
    const rclcpp::Node::SharedPtr& node, const std::string& service_name,
    const std::string& file_path, bool expected_success) {
  auto client = node->create_client<grid_map_msgs::srv::ProcessFile>(
      service_name);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<grid_map_msgs::srv::ProcessFile::Request>();
  request->file_path = file_path;
  request->topic_name = "terrain_map_raw";

  auto future = client->async_send_request(request);
  const auto result =
      rclcpp::spin_until_future_complete(node, future,
                                         std::chrono::seconds(3));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(static_cast<bool>(future.get()->success), expected_success);
}

bool spinUntilGridMap(
    const rclcpp::Node::SharedPtr& converter_node,
    const rclcpp::Node::SharedPtr& listener_node,
    const std::shared_ptr<grid_map_msgs::msg::GridMap>& msg,
    const std::chrono::milliseconds timeout = std::chrono::milliseconds(750)) {
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !msg &&
         std::chrono::steady_clock::now() - start < timeout) {
    rclcpp::spin_some(converter_node);
    rclcpp::spin_some(listener_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return msg != nullptr;
}

template <typename ConverterT>
void runConverterServiceTest(const std::string& node_name,
                             const std::string& ns) {
  auto node = std::make_shared<rclcpp::Node>(node_name, ns, converterOptions());
  ConverterT converter(node);
  auto listener_node =
      std::make_shared<rclcpp::Node>(node_name + "_listener", ns);
  std::shared_ptr<grid_map_msgs::msg::GridMap> published_map;
  auto map_sub = listener_node->create_subscription<grid_map_msgs::msg::GridMap>(
      "terrain_map_raw", rclcpp::QoS(1).transient_local(),
      [&](grid_map_msgs::msg::GridMap::SharedPtr msg) {
        published_map = msg;
      });

  EXPECT_EQ(converter.last_grid_map_ptr_, nullptr);

  expectProcessFileService<ConverterT>(node, "save_grid_map_to_file", "",
                                       false);
  expectProcessFileService<ConverterT>(node, "load_mesh_from_file", "",
                                       false);
  expectProcessFileService<ConverterT>(node, "load_mesh_from_file",
                                       flatMeshPath(), true);

  ASSERT_NE(converter.last_grid_map_ptr_, nullptr);
  EXPECT_TRUE(converter.last_grid_map_ptr_->exists("elevation"));
  EXPECT_TRUE(converter.last_grid_map_ptr_->exists("x"));
  EXPECT_TRUE(converter.last_grid_map_ptr_->exists("y"));
  EXPECT_EQ(converter.last_grid_map_ptr_->getFrameId(), "map");
  EXPECT_GT(converter.last_grid_map_ptr_->getSize().prod(), 0);

  ASSERT_TRUE(spinUntilGridMap(node, listener_node, published_map));
  EXPECT_EQ(published_map->header.frame_id, "map");
  EXPECT_FALSE(published_map->layers.empty());
}

}  // namespace

TEST(GridMapConvertersTest, MeshConverterServicesLoadAndRejectBadRequests) {
  runConverterServiceTest<mesh_to_grid_map::MeshToGridMapConverter>(
      "mesh_to_grid_map_node", "/mesh_converter_test");
}

TEST(GridMapConvertersTest, MjcfConverterServicesLoadAndRejectBadRequests) {
  runConverterServiceTest<mjcf_to_grid_map::MjcfToGridMapConverter>(
      "mjcf_to_grid_map_node", "/mjcf_converter_test");
}
