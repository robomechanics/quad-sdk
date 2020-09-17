#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include <ros/ros.h>
#include <spirit_msgs/BodyPlan.h>
#include <spirit_msgs/Footstep.h>
#include <spirit_msgs/FootstepPlan.h>

//! A template class for spirit
/*!
   RVizInterface is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class RVizInterface {
public:
	/**
	 * @brief Constructor for RVizInterface Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type RVizInterface
	 */
	RVizInterface(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
     * @brief Callback function to handle new body plan data
     * @param[in] Body plan message contining output of body planner
     */
    void bodyPlanCallback(const spirit_msgs::BodyPlan::ConstPtr& msg);

    /**
     * @brief Callback function to handle new footstep plan data
     * @param[in] Footstep plan message containing output of footstep planner
     */
    void footstepPlanCallback(const spirit_msgs::FootstepPlan::ConstPtr& msg);

	/// ROS subscriber for the body plan
	ros::Subscriber body_plan_sub_;

	/// ROS subscriber for the footstep plan
	ros::Subscriber footstep_plan_sub_;

	/// ROS Publisher for the body plan vizualization
	ros::Publisher body_plan_viz_pub_;

	/// ROS Publisher for the footstep plan visualization
	ros::Publisher footstep_plan_viz_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;
};

#endif // RVIZ_INTERFACE_H
