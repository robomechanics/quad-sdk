#ifndef MESH_TO_GRID_MAP_CONVERTER_H
#define MESH_TO_GRID_MAP_CONVERTER_H

#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_ros/point_cloud.h>

#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_core/GridMap.hpp>
#include <ros/package.h>

namespace mesh_to_grid_map {

constexpr double kDefaultGridMapResolution = 0.2;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultVerbose = false;
static const std::string kDefaultFrameIdMeshLoaded = "map";
static const std::string kDefaultWorldName = "flat";

class MeshToGridMapConverter {
 public:
  MeshToGridMapConverter(ros::NodeHandle nh, ros::NodeHandle nh_private);

 private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // Datacallback
  void meshCallback(const pcl_msgs::PolygonMesh& mesh);

  // Save callback
  bool saveGridMapService(grid_map_msgs::ProcessFile::Request& request,
                          grid_map_msgs::ProcessFile::Response& response);

  // Load mesh, service call
  bool loadMeshService(grid_map_msgs::ProcessFile::Request& request,
                       grid_map_msgs::ProcessFile::Response& response);

  // Load mesh from file
  bool loadMeshFromFile(const std::string& path_to_mesh_to_load);

  // Converts a mesh to grid map and stores the result
  bool meshToGridMap(const pcl::PolygonMesh& polygon_mesh,
                     const std::string& mesh_frame_id,
                     const uint64_t& time_stamp_nano_seconds);

  // Saves the grid map
  bool saveGridMap(const grid_map::GridMap& map,
                   const std::string& path_to_file,
                   const std::string& topic_name);

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Data subscribers.
  ros::Subscriber mesh_sub_;

  // Publishers
  ros::Publisher grid_map_pub_;

  // Services
  ros::ServiceServer save_grid_map_srv_;

  // Last grid map
  std::unique_ptr<grid_map::GridMap> last_grid_map_ptr_;

  // Grid Map Parameters
  double grid_map_resolution_;
  std::string layer_name_;
  std::string world_name_;

  // Control Parameters
  bool latch_grid_map_pub_;
  bool verbose_;

  // Load mesh parameters
  ros::ServiceServer load_map_service_server_;
  std::string frame_id_mesh_loaded_;
};

}  // namespace mesh_to_grid_map

#endif
