#include "spirit_utils/terrain_map_publisher.h"
#include <string>

TerrainMapPublisher::TerrainMapPublisher(ros::NodeHandle nh)
  : terrain_map_(grid_map::GridMap({"elevation"}))
{
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, image_topic;

  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("map_frame",map_frame_,"/map");
  nh.param<double>("terrain_map_publisher/update_rate", update_rate_, 10);
  nh.param<std::string>("terrain_map_publisher/map_data_source", map_data_source_, "internal");

  // Setup pubs and subs
  terrain_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(terrain_map_topic,1);

  // Add image subscriber if data source requests an image
  if (map_data_source_.compare("image")==0) {
    nh_.param<std::string>("topics/image", image_topic, "/image_publisher/image");
    nh_.param<double>("terrain_map_publisher/resolution", resolution_, 0.2);
    nh_.param<double>("terrain_map_publisher/min_height", min_height_, 0.0);
    nh_.param<double>("terrain_map_publisher/max_height", max_height_, 1.0);
    image_sub_ = nh_.subscribe(image_topic, 1, &TerrainMapPublisher::loadMapFromImage, this);
  }

  // Initialize the elevation layer on the terrain map
  terrain_map_.setBasicLayers({"elevation"});
}

void TerrainMapPublisher::createMap() {

  // Set initial map parameters and geometry
  terrain_map_.setFrameId(map_frame_);
  terrain_map_.setGeometry(grid_map::Length(12.0, 5.0), 0.2, grid_map::Position(4.0, 0.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    terrain_map_.getLength().x(), terrain_map_.getLength().y(),
    terrain_map_.getSize()(0), terrain_map_.getSize()(1));

  // Add an obstacle
  double obs_center[] = {2,0};
  double obs_radius = 0.5;
  for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {

    grid_map::Position position;
    terrain_map_.getPosition(*it, position);
    double x_diff = position.x() - obs_center[0];
    double y_diff = position.y() - obs_center[1];

    if (x_diff*x_diff + y_diff*y_diff <= obs_radius*obs_radius)
    {
      terrain_map_.at("elevation", *it) = 0.05;
    } else {
      terrain_map_.at("elevation", *it) = 0.0;
    }
  }
}

void TerrainMapPublisher::publishMap() {
  // Set the time at which the map was published
  ros::Time time = ros::Time::now();
  terrain_map_.setTimestamp(time.toNSec());

  // Generate grid_map message, convert, and publish
  grid_map_msgs::GridMap terrain_map_msg;
  grid_map::GridMapRosConverter::toMessage(terrain_map_, terrain_map_msg);
  terrain_map_pub_.publish(terrain_map_msg);
}

void TerrainMapPublisher::loadMapFromImage(const sensor_msgs::Image& msg) {

  // Initialize the map from the image message if not already done so
  if (!map_initialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, terrain_map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", terrain_map_.getLength().x(),
             terrain_map_.getLength().y(), terrain_map_.getSize()(0), terrain_map_.getSize()(1));
    map_initialized_ = true;
  }

  // Add the data layers
  grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", terrain_map_, min_height_, max_height_);
  grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", terrain_map_);

  // Move the map to place starting location at (0,0)
  grid_map::Position offset = {4.5,0.0};
  terrain_map_.setPosition(offset);
}

void TerrainMapPublisher::spin() {
  ros::Rate r(update_rate_);

  // Either wait for an image to show up on the topic or create a map from scratch
  if (map_data_source_.compare("image")==0) {
    // Spin until image message has been received and processed
    boost::shared_ptr<sensor_msgs::Image const> shared_image;
    while((shared_image == nullptr) && ros::ok())
    {
      shared_image = ros::topic::waitForMessage<sensor_msgs::Image>("/image_publisher/image", nh_);
      ros::spinOnce();
    }
  } else {
    createMap();
  }

  // Continue publishing the map at the update rate
  while (ros::ok()) {
    publishMap();
    ros::spinOnce();
    r.sleep();
  }
}