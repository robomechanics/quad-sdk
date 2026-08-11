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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <string>
#include <memory>
#include <vector>

namespace mesh_to_grid_map {

constexpr double kDefaultGridMapResolution = 0.02;
static const std::string kDefaultLayerName = "elevation";
constexpr bool kDefaultLatchGridMapPub = true;
constexpr bool kDefaultVerbose = true;
static const std::string kDefaultFrameIdMeshLoaded = "map";
static const std::string kDefaultWorldName = "flat";

// Mocap tracking of the terrain mesh. Empty track_frame keeps the historical
// behavior: the mesh is placed once using mesh_pose and never moves.
static const std::string kDefaultTrackFrame = "";
constexpr double kDefaultTrackRate = 5.0;               // Hz
constexpr double kDefaultTrackPositionTolerance = 0.005;    // m
constexpr double kDefaultTrackOrientationTolerance = 0.0087;  // rad (~0.5 deg)
// If the tracked frame never appears, fall back to mesh_pose after this long so
// downstream planners get terrain instead of nothing at all.
constexpr double kDefaultTrackTimeout = 5.0;  // s

class MeshToGridMapConverter {
 public:
  MeshToGridMapConverter(rclcpp::Node::SharedPtr node);

 private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // Datacallback
  void meshCallback(const pcl_msgs::msg::PolygonMesh::SharedPtr mesh);

  // Save callback
  bool saveGridMapService(
      const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
      std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  // Load mesh, service call
  bool loadMeshService(
      const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
      std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  // Load mesh from file
  bool loadMeshFromFile(const std::string& path_to_mesh_to_load);

  // Converts a mesh to grid map and stores the result
  bool meshToGridMap(const pcl::PolygonMesh& polygon_mesh,
                     const std::string& mesh_frame_id,
                     const uint64_t& time_stamp_nano_seconds);

  // Rigidly transforms the stored mesh into frame_id_mesh_loaded_ and
  // regenerates the grid map from it.
  bool publishMeshAtPose(const Eigen::Isometry3d& mesh_to_map);

  // Polls TF for the tracked frame and republishes when the platform moves
  void trackTimerCallback();

  // Builds an isometry from the mesh_pose parameter (x y z roll pitch yaw)
  Eigen::Isometry3d staticMeshPose() const;

  // Saves the grid map
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
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr
      save_grid_map_srv_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr
      load_map_service_server_;

  // Last grid map
  std::shared_ptr<grid_map::GridMap> last_grid_map_ptr_;

  // Mesh as loaded from file, kept in its own (platform-local) frame so it can
  // be re-placed whenever the tracked pose changes.
  pcl::PolygonMesh mesh_from_file_;
  bool mesh_loaded_{false};

  // Mocap tracking
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr track_timer_;
  Eigen::Isometry3d last_published_pose_{Eigen::Isometry3d::Identity()};
  bool has_published_pose_{false};
  rclcpp::Time track_start_time_;
  bool track_fallback_used_{false};

  // Grid Map Parameters
  double grid_map_resolution_;
  std::string layer_name_;
  bool latch_grid_map_pub_;
  bool verbose_;
  std::string frame_id_mesh_loaded_;
  std::string world_name_;

  // Tracking Parameters
  std::string track_frame_;
  double track_rate_;
  double track_position_tolerance_;
  double track_orientation_tolerance_;
  double track_timeout_;
  std::vector<double> mesh_pose_;

  // Control Parameters
};

}  // namespace mesh_to_grid_map

#endif
