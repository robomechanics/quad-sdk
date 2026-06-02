// #include <pcl_ros/point_cloud.h> // might cause a conflict
// #include "grid_map_msgs/msg/grid_map.hpp"
// #include "grid_map_msgs/srv/ProcessFile.hpp"
// #include <grid_map_pcl/GridMapPclConverter.hpp>
// #include <grid_map_ros/grid_map_ros.hpp>

// #include <pcl/io/vtk_lib_io.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include "ament_index_cpp/get_package_share_directory.hpp"

#include <quad_utils/mesh_to_grid_map_converter.hpp>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_msgs/srv/process_file.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_pcl/GridMapPclConverter.hpp>

#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "quad_utils/mesh_to_grid_map_converter.hpp"

namespace mesh_to_grid_map {

MeshToGridMapConverter::MeshToGridMapConverter(rclcpp::Node::SharedPtr node)
    : node_(node),
      grid_map_resolution_(kDefaultGridMapResolution),
      layer_name_(kDefaultLayerName),
      latch_grid_map_pub_(kDefaultLatchGridMapPub),
      verbose_(kDefaultVerbose),
      frame_id_mesh_loaded_(kDefaultFrameIdMeshLoaded),
      world_name_(kDefaultWorldName) {
  // Initial interaction with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();

  std::string package_path =
      ament_index_cpp::get_package_share_directory("quad_sim_scripts");
  // std::string package_path = ros::package::getPath("quad_sim_scripts");
  std::string base_name = world_name_;
  std::string extension = ".sdf";
  if (base_name.size() >= extension.size() &&
      base_name.compare(base_name.size() - extension.size(), extension.size(),
                        extension) == 0) {
    base_name = base_name.substr(0, base_name.size() - extension.size());
  }
  // std::cout << base_name <<std::endl;
  // std::cout << world_name_ << std::endl;
  std::string full_path =
      package_path + "/models/" + base_name + "/meshes/" + base_name + ".ply";

  RCLCPP_INFO(node_->get_logger(), "Loading mesh from: %s", full_path.c_str());
  bool success = loadMeshFromFile(full_path);
}

void MeshToGridMapConverter::subscribeToTopics() {  // UPDATED
  // mesh_sub_ =
  //     nh_.subscribe("mesh", 10, &MeshToGridMapConverter::meshCallback, this);
  mesh_sub_ = node_->create_subscription<pcl_msgs::msg::PolygonMesh>(
      "mesh", rclcpp::QoS(10),
      std::bind(&MeshToGridMapConverter::meshCallback, this,
                std::placeholders::_1));
}

void MeshToGridMapConverter::advertiseTopics() {  // UPDATED
  rclcpp::QoS qos_profile(1);
  if (latch_grid_map_pub_) {
    qos_profile = qos_profile.transient_local();
  }
  grid_map_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(
      "terrain_map_raw", qos_profile);
  // grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("terrain_map_raw", 1,
  //                                                       latch_grid_map_pub_);
}

void MeshToGridMapConverter::advertiseServices() {  // UPDATED
  save_grid_map_srv_ = node_->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_grid_map_to_file",
      std::bind(&MeshToGridMapConverter::saveGridMapService, this,
                std::placeholders::_1, std::placeholders::_2));

  load_map_service_server_ =
      node_->create_service<grid_map_msgs::srv::ProcessFile>(
          "load_mesh_from_file",
          std::bind(&MeshToGridMapConverter::loadMeshService, this,
                    std::placeholders::_1, std::placeholders::_2));

  // save_grid_map_srv_ = nh_private_.advertiseService(
  //     "save_grid_map_to_file", &MeshToGridMapConverter::saveGridMapService,
  //     this);
  // load_map_service_server_ = nh_private_.advertiseService(
  //     "load_mesh_from_file", &MeshToGridMapConverter::loadMeshService, this);
}

void MeshToGridMapConverter::getParametersFromRos() {  // UPDATED
  node_->declare_parameter("grid_map_resolution", grid_map_resolution_);
  node_->get_parameter("grid_map_resolution", grid_map_resolution_);

  node_->declare_parameter("layer_name", layer_name_);
  node_->get_parameter("layer_name", layer_name_);

  node_->declare_parameter("latch_grid_map_pub", latch_grid_map_pub_);
  node_->get_parameter("latch_grid_map_pub", latch_grid_map_pub_);

  node_->declare_parameter("verbose", verbose_);
  node_->get_parameter("verbose", verbose_);

  node_->declare_parameter("frame_id_mesh_loaded", frame_id_mesh_loaded_);
  node_->get_parameter("frame_id_mesh_loaded", frame_id_mesh_loaded_);

  node_->declare_parameter("world", world_name_);
  node_->get_parameter("world", world_name_);

  // nh_private_.param("grid_map_resolution", grid_map_resolution_,
  //                   grid_map_resolution_);
  // nh_private_.param("layer_name", layer_name_, layer_name_);
  // nh_private_.param("latch_grid_map_pub", latch_grid_map_pub_,
  //                   latch_grid_map_pub_);
  // nh_private_.param("verbose", verbose_, verbose_);
  // nh_private_.param("frame_id_mesh_loaded", frame_id_mesh_loaded_,
  //                   frame_id_mesh_loaded_);
  // nh_private_.param("world", world_name_, world_name_);
}

void MeshToGridMapConverter::meshCallback(  // UPDATED
    const pcl_msgs::msg::PolygonMesh::SharedPtr mesh_msg) {
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Mesh received, starting conversion.");
  }

  // Converting from message to an object
  pcl::PolygonMesh polygon_mesh;
  pcl_conversions::toPCL(*mesh_msg, polygon_mesh);
  meshToGridMap(polygon_mesh, mesh_msg->header.frame_id,
                mesh_msg->header.stamp.nanosec);
}

bool MeshToGridMapConverter::meshToGridMap(
    const pcl::PolygonMesh& polygon_mesh, const std::string& mesh_frame_id,
    const uint64_t& time_stamp_nano_seconds) {
  // Creating the grid map
  grid_map::GridMap map;
  map.setFrameId(mesh_frame_id);

  // Converting
  grid_map::GridMapPclConverter::initializeFromPolygonMesh(
      polygon_mesh, grid_map_resolution_, map);
  const std::string layer_name(layer_name_);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(polygon_mesh,
                                                         layer_name, map);

  // Setup x and y matrices for loading
  grid_map::Size map_size = map.getSize();
  Eigen::MatrixXf x_data(map_size(0), map_size(1)),
      y_data(map_size(0), map_size(1));

  // Iterate through map to retrieve x and y data and save to data matrices
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
       ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    map.getPosition(index, pos);
    x_data(index(0), index(1)) = pos(0);
    y_data(index(0), index(1)) = pos(1);
  }

  // Add x and y layers to map
  map.add("x", x_data);
  map.add("y", y_data);

  // Check on the Values of the Z layer
  //  if (map.exists("z")) {
  //      std::vector<grid_map::Index> corners = {
  //          grid_map::Index(0, 0),
  //          grid_map::Index(map.getSize()(0) - 1, 0),
  //          grid_map::Index(0, map.getSize()(1) - 1),
  //          grid_map::Index(map.getSize()(0) - 1, map.getSize()(1) - 1)
  //      };

  //     for (size_t i = 0; i < corners.size(); ++i) {
  //         const auto& index = corners[i];
  //         try {
  //             float z = map.at("z", index);
  //             std::cout << "Corner " << i << " at index (" << index(0) << ",
  //             " << index(1)
  //                       << ") has z = " << z << std::endl;
  //         } catch (const std::exception& e) {
  //             std::cerr << "Error accessing z at corner " << i << ": " <<
  //             e.what() << std::endl;
  //         }
  //     }
  // }

  // Printing some debug info about the mesh and the map
  if (verbose_) {
    // RCLCPP_INFO_STREAM("Number of polygons: " <<
    // polygon_mesh.polygons.size());
    RCLCPP_INFO(node_->get_logger(), "Created map %f x %f m (%i x %i cells)",
                map.getLength().x(), map.getLength().y(), map.getSize()(0),
                map.getSize()(1));
  }

  // Publish grid map.
  map.setTimestamp(time_stamp_nano_seconds);
  // grid_map_msgs::msg::GridMap message_ptr;
  auto message_ptr = grid_map::GridMapRosConverter::toMessage(map);

  // Publishing the grid map message.
  grid_map_pub_->publish(*message_ptr);
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Published a grid map message.");
  }

  // Saving the gridmap to the object
  // last_grid_map_ptr_.reset(new grid_map::GridMap(map));
  last_grid_map_ptr_ = std::make_shared<grid_map::GridMap>(map);

  return true;
}

bool MeshToGridMapConverter::saveGridMapService(  // UPDATED
    const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
    std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  // Check there's actually a grid map saved
  if (!last_grid_map_ptr_) {
    RCLCPP_ERROR(node_->get_logger(), "No grid map produced yet to save.");
    response->success = static_cast<unsigned char>(false);
  } else {
    response->success = static_cast<unsigned char>(saveGridMap(
        *last_grid_map_ptr_, request->file_path, request->topic_name));
  }

  return true;
}

bool MeshToGridMapConverter::saveGridMap(
    const grid_map::GridMap& map, const std::string& path_to_file,
    const std::string& topic_name) {  // UPDATED
  std::string topic_name_checked = topic_name;
  if (topic_name.empty()) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Specified topic name is an empty string, default layer name will be "
        "used as topic name.");
    topic_name_checked = layer_name_;
  }
  // Saving the map
  if (!path_to_file.empty()) {
    if (verbose_) {
      RCLCPP_INFO(
          node_->get_logger(),
          "Saved the grid map message to file: '%s', with topic name: '%s'.",
          path_to_file.c_str(), topic_name_checked.c_str());
    }
    grid_map::GridMapRosConverter::saveToBag(map, path_to_file,
                                             topic_name_checked);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "No rosbag filepath specified where to save grid map.");
    return false;
  }
  return true;
}

bool MeshToGridMapConverter::loadMeshService(  // UPDATED
    std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
    std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  if (!request->topic_name.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Field 'topic_name' in service request will not be used.");
  }
  response->success =
      static_cast<unsigned char>(loadMeshFromFile(request->file_path));
  return true;
}

bool MeshToGridMapConverter::loadMeshFromFile(
    const std::string& path_to_mesh_to_load) {
  if (path_to_mesh_to_load.empty()) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "File path for mesh to load is empty. Please specify a valid path.");
    return false;
  }

  pcl::PolygonMesh mesh_from_file;
  pcl::io::loadPolygonFilePLY(path_to_mesh_to_load, mesh_from_file);

  if (mesh_from_file.polygons.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Mesh read from file is empty!");
    return false;
  }

  bool mesh_converted = meshToGridMap(mesh_from_file, frame_id_mesh_loaded_,
                                      node_->now().nanoseconds());
  if (!mesh_converted) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "It was not possible to convert loaded mesh to grid_map object.");
    return false;
  }

  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(),
                "Loaded the mesh from file: %s. Its frame_id is set to '%s'",
                path_to_mesh_to_load.c_str(), frame_id_mesh_loaded_.c_str());
  }

  return true;
}

}  // namespace mesh_to_grid_map
