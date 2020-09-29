/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "spirit_utils/ImageToGridmapDemoCustom.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>


ImageToGridmapDemo::ImageToGridmapDemo(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(grid_map::GridMap({"elevation"})),
      mapInitialized_(false)
{
  readParameters();
  map_.setBasicLayers({"elevation"});
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &ImageToGridmapDemo::imageCallback, this);
  gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/terrain_map", 1, true);
}

ImageToGridmapDemo::~ImageToGridmapDemo()
{
}

bool ImageToGridmapDemo::readParameters()
{
  nodeHandle_.param("image_to_gridmap_demo_custom_node/image_topic", imageTopic_, std::string("/image_publisher/image"));
  nodeHandle_.param("image_to_gridmap_demo_custom_node/resolution", resolution_, 0.2);
  nodeHandle_.param("image_to_gridmap_demo_custom_node/min_height", minHeight_, 0.0);
  nodeHandle_.param("image_to_gridmap_demo_custom_node/max_height", maxHeight_, 1.05);

  std::cout << "Max height set to " << maxHeight_ << std::endl;
  return true;
}

void ImageToGridmapDemo::imageCallback(const sensor_msgs::Image& msg)
{
  if (!mapInitialized_) {
    grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells), resolution_ %f.", map_.getLength().x(),
             map_.getLength().y(), map_.getSize()(0), map_.getSize()(1), resolution_);
    mapInitialized_ = true;
  }


  grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);
  grid_map::Position offset = {4.5,0.0};
  map_.setPosition(offset);
  // grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);

  // interpolatedMap_ = createInterpolatedMapFromDataMap(map_, 0.1);
  // interpolateInputMap(map_, grid_map::InterpolationMethods::INTER_LINEAR,
  //                     &interpolatedMap_);


  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
  gridMapPublisher_.publish(mapMessage);
}
