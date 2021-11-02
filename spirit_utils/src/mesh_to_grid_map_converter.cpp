#include <spirit_utils/mesh_to_grid_map_converter.hpp>

#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
namespace mesh_to_grid_map {

MeshToGridMapConverter::MeshToGridMapConverter(ros::NodeHandle nh,
                                               ros::NodeHandle nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      grid_map_resolution_(kDefaultGridMapResolution),
      layer_name_(kDefaultLayerName),
      latch_grid_map_pub_(kDefaultLatchGridMapPub),
      verbose_(kDefaultVerbose),
      frame_id_mesh_loaded_(kDefaultFrameIdMeshLoaded),
      world_name_(kDefaultWorldName) {
  // Initial interaction with ROS
  subscribeToTopics();
  advertiseTopics();
  advertiseServices();
  getParametersFromRos();
  loadParams();

  std::string package_path = ros::package::getPath("gazebo_scripts");
  std::string full_path = package_path + "/worlds/" + world_name_ + "/" + world_name_ + ".obj";

  generateTerrain(full_path);

  while(!exists_file(full_path)){};
  std::cout << full_path << std::endl;
  bool success = loadMeshFromFile(full_path);

}

void MeshToGridMapConverter::loadParams() {
  nh_.param<double>("/mesh_to_grid_map_converter/numPointsX", profile_.numPointsX, 100.0);
  nh_.param<double>("/mesh_to_grid_map_converter/numPointsY", profile_.numPointsY, 100.0);
  nh_.param<double>("/mesh_to_grid_map_converter/waviness", profile_.waviness, 0.0);
  nh_.param<double>("/mesh_to_grid_map_converter/xMin", profile_.xMin, -10.0);
  nh_.param<double>("/mesh_to_grid_map_converter/yMin", profile_.yMin, -10.0);
  nh_.param<double>("/mesh_to_grid_map_converter/zMin", profile_.zMin, 0.0);
  nh_.param<double>("/mesh_to_grid_map_converter/xMax", profile_.xMax, 10.0);
  nh_.param<double>("/mesh_to_grid_map_converter/yMax", profile_.yMax, 10.0);
  nh_.param<double>("/mesh_to_grid_map_converter/zMax", profile_.zMax, 0.0);
  nh_.param<int>("/mesh_to_grid_map_converter/seed", seed, 1);
  std::cout<< profile_.numPointsX << profile_.numPointsY << profile_.waviness<<std::endl;
}

inline bool MeshToGridMapConverter::exists_file (const std::string& name) {
  ifstream f(name.c_str());
  return f.good();
}

void MeshToGridMapConverter::subscribeToTopics() {
  mesh_sub_ =
      nh_.subscribe("mesh", 10, &MeshToGridMapConverter::meshCallback, this);
}

void MeshToGridMapConverter::advertiseTopics() {
  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(
      "terrain_map_raw", 1, latch_grid_map_pub_);
}

void MeshToGridMapConverter::advertiseServices() {
  save_grid_map_srv_ = nh_private_.advertiseService(
      "save_grid_map_to_file", &MeshToGridMapConverter::saveGridMapService, this);
  load_map_service_server_ = nh_private_.advertiseService(
      "load_mesh_from_file", &MeshToGridMapConverter::loadMeshService, this);
}

void MeshToGridMapConverter::getParametersFromRos() {
  nh_private_.param("grid_map_resolution", grid_map_resolution_,
                    grid_map_resolution_);
  nh_private_.param("layer_name", layer_name_, layer_name_);
  nh_private_.param("latch_grid_map_pub", latch_grid_map_pub_,
                    latch_grid_map_pub_);
  nh_private_.param("verbose", verbose_, verbose_);
  nh_private_.param("frame_id_mesh_loaded", frame_id_mesh_loaded_,
                    frame_id_mesh_loaded_);
  nh_private_.param("world", world_name_,
                    world_name_);
}

void MeshToGridMapConverter::meshCallback(
    const pcl_msgs::PolygonMesh& mesh_msg) {
  if (verbose_) {
    ROS_INFO("Mesh received, starting conversion.");
  }

  // Converting from message to an object
  pcl::PolygonMesh polygon_mesh;
  pcl_conversions::toPCL(mesh_msg, polygon_mesh);
  meshToGridMap(polygon_mesh, mesh_msg.header.frame_id,
                mesh_msg.header.stamp.toNSec());
}

bool MeshToGridMapConverter::meshToGridMap(
    const pcl::PolygonMesh& polygon_mesh, const std::string& mesh_frame_id,
    const uint64_t& time_stamp_nano_seconds) {
  // Creating the grid map
  grid_map::GridMap map;
  map.setFrameId(mesh_frame_id);

  // Converting
  grid_map::GridMapPclConverter::initializeFromPolygonMesh(polygon_mesh,
      grid_map_resolution_, map);
  const std::string layer_name(layer_name_);
  grid_map::GridMapPclConverter::addLayerFromPolygonMesh(polygon_mesh,
      layer_name, map);

  // Printing some debug info about the mesh and the map
  if (verbose_) {
    ROS_INFO_STREAM("Number of polygons: " << polygon_mesh.polygons.size());
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(), map.getSize()(0),
             map.getSize()(1));
  }

  // Publish grid map.
  map.setTimestamp(time_stamp_nano_seconds);
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);

  // Publishing the grid map message.
  grid_map_pub_.publish(message);
  if (verbose_) {
    ROS_INFO("Published a grid map message.");
  }

  // Saving the gridmap to the object
  last_grid_map_ptr_.reset(new grid_map::GridMap(map));

  return true;
}

bool MeshToGridMapConverter::saveGridMapService(grid_map_msgs::ProcessFile::Request& request,
                                                grid_map_msgs::ProcessFile::Response& response) {
  // Check there's actually a grid map saved
  if (!last_grid_map_ptr_) {
    ROS_ERROR("No grid map produced yet to save.");
    response.success = static_cast<unsigned char>(false);
  } else {
    response.success = static_cast<unsigned char>(saveGridMap(*last_grid_map_ptr_,
        request.file_path, request.topic_name));
  }

  return true;
}

bool MeshToGridMapConverter::saveGridMap(const grid_map::GridMap& map,
                                         const std::string& path_to_file,
                                         const std::string& topic_name) {
  std::string topic_name_checked = topic_name;
  if (topic_name.empty()) {
    ROS_WARN("Specified topic name is an empty string, default layer name will be used as topic name.");
    topic_name_checked = layer_name_;
  }
  // Saving the map
  if (!path_to_file.empty()) {
    if (verbose_) {
      ROS_INFO(
          "Saved the grid map message to file: '%s', with topic name: '%s'.",
          path_to_file.c_str(), topic_name_checked.c_str());
    }
    grid_map::GridMapRosConverter::saveToBag(map, path_to_file,
                                             topic_name_checked);
  } else {
    ROS_ERROR(
        "No rosbag filepath specified where to save grid map.");
    return false;
  }
  return true;
}

bool MeshToGridMapConverter::loadMeshService(grid_map_msgs::ProcessFile::Request& request,
                                             grid_map_msgs::ProcessFile::Response& response) {
  if (!request.topic_name.empty()) {
    ROS_WARN("Field 'topic_name' in service request will not be used.");
  }
  response.success = static_cast<unsigned char>(loadMeshFromFile(request.file_path));
  return true;
}

bool MeshToGridMapConverter::loadMeshFromFile(const std::string& path_to_mesh_to_load) {
  if (path_to_mesh_to_load.empty()) {
    ROS_ERROR(
        "File path for mesh to load is empty. Please specify a valid path.");
    return false;
  }

  pcl::PolygonMesh mesh_from_file;
  pcl::io::loadPolygonFileOBJ(path_to_mesh_to_load,
                              mesh_from_file);

  if (mesh_from_file.polygons.empty()) {
    ROS_ERROR("Mesh read from file is empty!");
    return false;
  }

  bool mesh_converted = meshToGridMap(mesh_from_file, frame_id_mesh_loaded_,
                                      ros::Time::now().toNSec());
  if (!mesh_converted) {
    ROS_ERROR("It was not possible to convert loaded mesh to grid_map object.");
    return false;
  }

  if (verbose_) {
    ROS_INFO("Loaded the mesh from file: %s. Its frame_id is set to '%s'",
        path_to_mesh_to_load.c_str(), frame_id_mesh_loaded_.c_str());
  }

  return true;
}

void MeshToGridMapConverter::GetInputPoints(std::vector<GEOM_FADE25D::Point2>& vPointsOut)
{
	GEOM_FADE25D::generateRandomSurfacePoints(
		 profile_.numPointsX, // profile_.numPointsX
		 profile_.numPointsY,
		 profile_.waviness, // numCenters (waviness)
		 profile_.xMin, profile_.yMin, profile_.zMin, profile_.xMax, profile_.yMax, profile_.zMax, /* Bounds xmin, ymin, zmin, xmax, ymax, zmax(0, 0, 0, 100, 100, 50) */
		vPointsOut, // Output vector
		seed  // Seed (0 is random, >0 is static)
	);
}

void MeshToGridMapConverter::generateTerrain(std::string path) {

  // // // diamond_square
  // double range = 4.0; // +-10
	// int width = pow(2, 8)+1;
  // double** map_ = new double* [width];
  // std::cout<< "here"<<std::endl;
	// for(int i = 0; i < width; i++)
	// {
	// 	map_[i] = new double[width];
	// 	for(int j = 0; j < width; j++)
	// 		map_[i][j] = 0.0;
	// }
  // std::cout<< "here"<<std::endl;
	// DiamondSquare ds(map_, width, range);
  // std::cout<< "here"<<std::endl;
	// double** map = ds.process();
  // std::cout<< "here"<<std::endl;
  // //std::cout<<map<<std::endl;

  // std::vector<GEOM_FADE25D::Point2> vInputPoints;
  // vInputPoints.reserve(width*width);
	// for(int x=0;x<=width-1;++x)
	// {
	// 	for(int y=0;y<=width-1;++y)
	// 	{
  //     // std::cout<<map[x][y]<<" "<<std::endl; 
	// 		vInputPoints.push_back(GEOM_FADE25D::Point2(x,y,map[x][y]));
	// 	}
	// }

	std::vector<GEOM_FADE25D::Point2> vInputPoints;
	GetInputPoints(vInputPoints);

	GEOM_FADE25D::Fade_2D dt;
	GEOM_FADE25D::CloudPrepare cloudPrep;
	cloudPrep.add(vInputPoints);
	dt.insert(&cloudPrep, true); // More memory efficient
  GEOM_FADE25D::Zone2* pGlobalZone(dt.createZone(NULL,GEOM_FADE25D::ZL_GLOBAL));
	pGlobalZone->smoothing(2);

	GEOM_FADE25D::FadeExport fadeExport;
	bool bCustomIndices(true);
	bool bClear(true);
	dt.exportTriangulation(fadeExport, bCustomIndices, bClear);

	dt.showGeomview((std::string("random")+std::string(".list")).c_str());
	fadeExport.writeObj(path.c_str());
}

}  // namespace mesh_to_grid_map
