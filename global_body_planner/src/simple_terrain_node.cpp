#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

using namespace grid_map;

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "simple_terrain");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("terrain_map", 1, true);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(12.0, 5.0), 0.2, Position(4.0, 0.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  ros::Rate rate(10.0);
  while (nh.ok()) {

    double obs_center[] = {2,0};
    double obs_radius = 0.5;
    // Add data to grid map.
    ros::Time time = ros::Time::now();
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      double x_diff = position.x() - obs_center[0];
      double y_diff = position.y() - obs_center[1];
      if (x_diff*x_diff + y_diff*y_diff <= obs_radius*obs_radius)
      {
        map.at("elevation", *it) = 0.5;
      } else {
        map.at("elevation", *it) = 0.0;
      }
      // map.at("elevation", *it) = 0.1*(*it);
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}
