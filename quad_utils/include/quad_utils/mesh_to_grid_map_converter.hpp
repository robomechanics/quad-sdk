#ifndef MESH_TO_GRID_MAP_CONVERTER_H
#define MESH_TO_GRID_MAP_CONVERTER_H

#include <rclcpp/rclcpp.hpp>


// #include <grid_map_msgs/srv/process_file.h>
// #include <pcl/PolygonMesh.h>
// #include <pcl_msgs/PolygonMesh.h>
// #include <pcl_ros/point_cloud.h>
// #include <ros/package.h>
// #include <ros/ros.h>
// #include <std_srvs/Empty.h>

#include <grid_map_msgs/srv/process_file.hpp>
#include <grid_map_core/GridMap.hpp>
#include <pcl/PolygonMesh.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl_msgs/msg/polygon_mesh.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <memory>

namespace mesh_to_grid_map {

constexpr double kDefaultGridMapResolution = 0.2;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultVerbose = true;
static const std::string kDefaultFrameIdMeshLoaded = "map";
static const std::string kDefaultWorldName = "flat";

class MeshToGridMapConverter {
 public:
  MeshToGridMapConverter(rclcpp::Node::SharedPtr node);

 private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  /** @brief Callback for incoming mesh messages
   * @param[in] mesh Shared pointer to the incoming mesh message
   */
  void meshCallback(const pcl_msgs::msg::PolygonMesh::SharedPtr mesh);

  /** @brief Callback for saving the grid map service
   * @param[in] request Shared pointer to the service request
   * @param[in] response Shared pointer to the service response
   * @return True if successful, false otherwise
   */
  bool saveGridMapService(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                          std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  /** @brief Callback for loading a mesh from file service
   * @param[in] request Shared pointer to the service request
   * @param[in] response Shared pointer to the service response
   * @return True if successful, false otherwise
   */
  bool loadMeshService(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                       std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  /** @brief Load a mesh from file
   * @param[in] path_to_mesh_to_load Path to the mesh file to load
   * @return True if successful, false otherwise
   */
  bool loadMeshFromFile(const std::string& path_to_mesh_to_load);

  /** @brief Convert a mesh to a grid map
   * @param[in] polygon_mesh The input polygon mesh
   * @param[in] mesh_frame_id The frame id of the input mesh
   * @param[in] time_stamp_nano_seconds The timestamp of the input mesh in nanoseconds
   * @return True if successful, false otherwise
   */
  bool meshToGridMap(const pcl::PolygonMesh& polygon_mesh,
                     const std::string& mesh_frame_id,
                     const uint64_t& time_stamp_nano_seconds);

  /** @brief Save the grid map to a file
   * @param[in] map The grid map to save
   * @param[in] path_to_file The path to the file where the grid map should be saved
   * @param[in] topic_name The topic name associated with the grid map
   * @return True if successful, false otherwise
   */
  bool saveGridMap(const grid_map::GridMap& map,
                   const std::string& path_to_file,
                   const std::string& topic_name);

  // Node Handles
  rclcpp::Node::SharedPtr node_;

  // Data subscribers.
//   ros::Subscriber mesh_sub_;
  rclcpp::Subscription<pcl_msgs::msg::PolygonMesh>::SharedPtr mesh_sub_;

  // Publishers
//   ros::Publisher grid_map_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;

  // Services
//   ros::ServiceServer save_grid_map_srv_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr save_grid_map_srv_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr load_map_service_server_;

  // Last grid map
  std::shared_ptr<grid_map::GridMap> last_grid_map_ptr_;

  // Grid Map Parameters
  double grid_map_resolution_;
  std::string layer_name_;
  bool latch_grid_map_pub_;
  bool verbose_;
  std::string frame_id_mesh_loaded_;
  std::string world_name_;

  // Control Parameters
  
};

}  // namespace mesh_to_grid_map

#endif
