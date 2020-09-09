#ifndef LOCAL_FOOTSTEP_PLANNER_H
#define LOCAL_FOOTSTEP_PLANNER_H

#include <ros/ros.h>

//! A local footstep planning class for spirit
/*!
   FootstepPlanner is a container for all of the logic utilized in the local footstep planning node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class LocalFootstepPlanner {
  public:
	/**
	 * @brief Constructor for LocalFootstepPlanner Class
	 * @param[in] nh ROS Nodehandle to publish and subscribe from
	 * @return Constructed object of type FootstepPlanner
	 */
	LocalFootstepPlanner(ros::NodeHandle nh);

	/**
	 * @brief Primary work function in class, called in node file for this component
	 */
	void loop();

  private:
	//! ROS Nodehandle to publish and subscribe from
	ros::NodeHandle _nh;
};


#endif // SPIRIT_COMPONENT_TEMPLATE_H