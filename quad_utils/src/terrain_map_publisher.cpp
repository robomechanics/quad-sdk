#include "quad_utils/terrain_map_publisher.h"

TerrainMapPublisher::TerrainMapPublisher(ros::NodeHandle nh)
  : terrain_map_(grid_map::GridMap({"z","nx","ny","nz",
    "z_filt","nx_filt","ny_filt","nz_filt"}))
{
  nh_ = nh;

  // Load rosparams from parameter server
  std::string terrain_map_topic, image_topic;

  nh.param<std::string>("topics/terrain_map", terrain_map_topic, "/terrain_map");
  nh.param<std::string>("map_frame",map_frame_,"map");
  nh.param<double>("terrain_map_publisher/update_rate", update_rate_, 10);
  nh.param<double>("terrain_map_publisher/obstacle_x", obstacle_.x, 2.0);
  nh.param<double>("terrain_map_publisher/obstacle_y", obstacle_.y, 0.0);
  nh.param<double>("terrain_map_publisher/obstacle_height", obstacle_.height, 0.5);
  nh.param<double>("terrain_map_publisher/obstacle_radius", obstacle_.radius, 1.0);
  nh.param<double>("terrain_map_publisher/step1_x", step1_.x, 4.0);
  nh.param<double>("terrain_map_publisher/step1_height", step1_.height, 0.3);
  nh.param<double>("terrain_map_publisher/step2_x", step2_.x, 4.0);
  nh.param<double>("terrain_map_publisher/step2_height", step2_.height, 0.3);
  nh.param<double>("terrain_map_publisher/resolution", resolution_, 0.2);
  nh.param<double>("terrain_map_publisher/update_rate", update_rate_, 10);
  nh.param<std::string>("terrain_map_publisher/map_data_source", map_data_source_, "internal");
  nh.param<std::string>("terrain_map_publisher/terrain_type", terrain_type_, "slope");
  // Setup pubs and subs
  terrain_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(terrain_map_topic,1);

  // Add image subscriber if data source requests an image
  if (map_data_source_.compare("image")==0) {
    nh_.param<std::string>("topics/image", image_topic, "/image_publisher/image");
    nh_.param<double>("terrain_map_publisher/min_height", min_height_, 0.0);
    nh_.param<double>("terrain_map_publisher/max_height", max_height_, 1.0);
    image_sub_ = nh_.subscribe(image_topic, 1, &TerrainMapPublisher::loadMapFromImage, this);
  }

  // Initialize the elevation layer on the terrain map
  terrain_map_.setBasicLayers({"z","nx","ny","nz",
    "z_filt","nx_filt","ny_filt","nz_filt"});

}

void TerrainMapPublisher::updateParams() {
  nh_.param<double>("terrain_map_publisher/obstacle_x", obstacle_.x, 2.0);
  nh_.param<double>("terrain_map_publisher/obstacle_y", obstacle_.y, 0.0);
  nh_.param<double>("terrain_map_publisher/obstacle_height", obstacle_.height, 0.5);
  nh_.param<double>("terrain_map_publisher/obstacle_radius", obstacle_.radius, 1.0);
  nh_.param<double>("terrain_map_publisher/step1_x", step1_.x, 4.0);
  nh_.param<double>("terrain_map_publisher/step1_height", step1_.height, 0.3);
  nh_.param<double>("terrain_map_publisher/step2_x", step2_.x, 6.0);
  nh_.param<double>("terrain_map_publisher/step2_height", step2_.height, -0.3);
}

void TerrainMapPublisher::createMap() {

  // Set initial map parameters and geometry
  terrain_map_.setFrameId(map_frame_);
  terrain_map_.setGeometry(grid_map::Length(12.0, 12.0), resolution_,
    grid_map::Position(-0.5*resolution_, -0.5*resolution_));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    terrain_map_.getLength().x(), terrain_map_.getLength().y(),
    terrain_map_.getSize()(0), terrain_map_.getSize()(1));

}

void TerrainMapPublisher::updateMap() {
  // Add terrain info
  for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {

    grid_map::Position position;
    terrain_map_.getPosition(*it, position);
    double x_diff = position.x() - obstacle_.x;
    double y_diff = position.y() - obstacle_.y;

    if (x_diff*x_diff + y_diff*y_diff <= obstacle_.radius*obstacle_.radius)
    {
      terrain_map_.at("z", *it) = obstacle_.height;
      terrain_map_.at("z_filt", *it) = obstacle_.height;
    } else {
      terrain_map_.at("z", *it) = 0.0;
      terrain_map_.at("z_filt", *it) = 0.0;
    }

    if (position.x() >= step1_.x) {
      terrain_map_.at("z", *it) += step1_.height;
      terrain_map_.at("z_filt", *it) += step1_.height;
    }

    if (position.x() >= step2_.x) {
      terrain_map_.at("z", *it) += step2_.height;
      terrain_map_.at("z_filt", *it) += step2_.height;
    }

    terrain_map_.at("nx", *it) = 0.0;
    terrain_map_.at("ny", *it) = 0.0;
    terrain_map_.at("nz", *it) = 1.0;

    terrain_map_.at("nx_filt", *it) = 0.0;
    terrain_map_.at("ny_filt", *it) = 0.0;
    terrain_map_.at("nz_filt", *it) = 1.0;
  }
}

std::vector<std::vector<double> > TerrainMapPublisher::loadCSV(std::string filename) {
  
  std::vector<std::vector<double> > data;
  std::ifstream inputFile(filename);
  int l = 0;

  while (inputFile) {
      l++;
      std::string s;
      if (!getline(inputFile, s)) break;
      if (s[0] != '#') {
          std::istringstream ss(s);
          std::vector<double> record;

          while (ss) {
              std::string line;
              if (!getline(ss, line, ','))
                  break;
              try {
                  record.push_back(stod(line));
              }
              catch (const std::invalid_argument e) {
                  std::cout << "NaN found in file " << filename << " line " << l
                       << std::endl;
                  e.what();
              }
          }

          data.push_back(record);
      }
  }

  if (!inputFile.eof()) {
      std::cerr << "Could not read file " << filename << "\n";
      std::__throw_invalid_argument("File not found.");
  }

  return data;
}

void TerrainMapPublisher::loadMapFromCSV() {

  // Load in all terrain data
  std::string package_path = ros::package::getPath("quad_utils");
  std::vector<std::vector<double> > x_data = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/x_data.csv");
  std::vector<std::vector<double> > y_data = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/y_data.csv");
  std::vector<std::vector<double> > z_data = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/z_data.csv");
  std::vector<std::vector<double> > nx_data = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/nx_data.csv");
  std::vector<std::vector<double> > ny_data = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/ny_data.csv");
  std::vector<std::vector<double> > nz_data = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/nz_data.csv");
  std::vector<std::vector<double> > z_data_filt = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/z_data_filt.csv");
  std::vector<std::vector<double> > nx_data_filt = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/nx_data_filt.csv");
  std::vector<std::vector<double> > ny_data_filt = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/ny_data_filt.csv");
  std::vector<std::vector<double> > nz_data_filt = loadCSV(package_path + 
    "/data/" + terrain_type_ + "/nz_data_filt.csv");

  // Grab map length and resolution parameters, make sure resolution is square (and align grid centers with data points)
  int x_size = z_data[0].size();
  int y_size = z_data.size();
  float x_res = x_data[0][1] - x_data[0][0];
  float y_res = y_data[1][0] - y_data[0][0];
  double x_length = x_data[0].back() - x_data[0].front() + x_res;
  double y_length = y_data.back()[0] - y_data.front()[0] + y_res;
  if (x_res != y_res) {
    throw std::runtime_error("Map did not have square elements, make sure x and y resolution are equal.");
  }

  // Initialize the map
  terrain_map_.setFrameId(map_frame_);
  terrain_map_.setGeometry(grid_map::Length(x_length, y_length), x_res, grid_map::Position(
    x_data[0].front()-0.5*x_res + 0.5*x_length, y_data.front()[0]-0.5*y_res + 0.5*y_length));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    terrain_map_.getLength().x(), terrain_map_.getLength().y(),
    terrain_map_.getSize()(0), terrain_map_.getSize()(1));

  // Load in the elevation and slope data
  for (grid_map::GridMapIterator iterator(terrain_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position position;
    terrain_map_.getPosition(*iterator, position);
    terrain_map_.at("z", *iterator) = z_data[(y_size-1) - index[1]][(x_size-1) - index[0]];
    terrain_map_.at("nx", *iterator) = nx_data[(y_size-1) - index[1]][(x_size-1) - index[0]];
    terrain_map_.at("ny", *iterator) = ny_data[(y_size-1) - index[1]][(x_size-1) - index[0]];
    terrain_map_.at("nz", *iterator) = nz_data[(y_size-1) - index[1]][(x_size-1) - index[0]];

    terrain_map_.at("z_filt", *iterator) = z_data_filt[(y_size-1) - index[1]][(x_size-1) - index[0]];
    terrain_map_.at("nx_filt", *iterator) = nx_data_filt[(y_size-1) - index[1]][(x_size-1) - index[0]];
    terrain_map_.at("ny_filt", *iterator) = ny_data_filt[(y_size-1) - index[1]][(x_size-1) - index[0]];
    terrain_map_.at("nz_filt", *iterator) = nz_data_filt[(y_size-1) - index[1]][(x_size-1) - index[0]];
  }
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
  grid_map::GridMapRosConverter::addLayerFromImage(msg, "z", terrain_map_, min_height_, max_height_);
  grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", terrain_map_);

  // Add in slope information
  for (grid_map::GridMapIterator it(terrain_map_); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    terrain_map_.at("nx", *it) = 0.0;
    terrain_map_.at("ny", *it) = 0.0;
    terrain_map_.at("nz", *it) = 1.0;
  }

  // Move the map to place starting location at (0,0)
  grid_map::Position offset = {4.5,0.0};
  terrain_map_.setPosition(offset);
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
  } else if (map_data_source_.compare("csv")==0) {
    loadMapFromCSV();
  } else {
    createMap();
  }

  // Continue publishing the map at the update rate
  while (ros::ok()) {

    updateParams();

    if (map_data_source_.compare("internal")==0) {
      updateMap();
    }
    
    publishMap();
    ros::spinOnce();
    r.sleep();
  }
}