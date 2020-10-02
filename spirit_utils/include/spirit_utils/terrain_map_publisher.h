#ifndef TERRAIN_MAP_PUBLISHER_H
#define TERRAIN_MAP_PUBLISHER_H

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

//! A terrain map publishing class
/*!
   TerrainMapPublisher is a class for publishing terrain maps from a variety of sources, including from scratch.
*/
class TerrainMapPublisher {
public:
	/**
	 * @brief Constructor for TerrainMapPublisher Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type TerrainMapPublisher
	 */
	TerrainMapPublisher(ros::NodeHandle nh);

	/**
	 * @brief Creates the map object from scratch
	 */
	void createMap();

	/**
	 * @brief Loads data into the map object from an image topic
	 */
	void loadMapFromImage(const sensor_msgs::Image& msg);

	/**
	 * @brief Publishes map data to the terrain_map topic
	 */
	void publishMap();

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:

	/// ROS Subscriber for image data
	ros::Subscriber image_sub_;

	/// ROS Publisher for the terrain map
	ros::Publisher terrain_map_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data, unused since pubs are called in callbacks
	double update_rate_;

	/// Handle for the map frame
    std::string map_frame_;

    /// grid_map::GridMap object for terrain data
    grid_map::GridMap terrain_map_;

    /// string of the source of the terrain map data
    std::string map_data_source_;

    /// bool to flag if the map has been initialized yet
    bool map_initialized_ = false;

    /// double for map resolution
    double resolution_;

    /// double for map resolution
    double min_height_;

    /// double for map resolution
    double max_height_;
};

#endif // TERRAIN_MAP_PUBLISHER_H
