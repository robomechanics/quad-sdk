#ifndef CONTACT_STATE_PUBLISHER_H
#define CONTACT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <spirit_msgs/GRFArray.h>
#include <gazebo_msgs/ContactsState.h>
#include <spirit_utils/ros_utils.h>


#include <cmath>
#define MATH_PI 3.141592


//! Publishes contact states from gazebo
/*!
   This class subscribes to Gazebo contact state messages and publishes their data under one GRFArray message.
*/
class ContactStatePublisher {
  public:
	/**
	 * @brief Constructor for ContactStatePublisher
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type ContactStatePublisher
	 */
	ContactStatePublisher(ros::NodeHandle nh);
	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();
  
private:
	/**
	 * @brief Processes new contact state data
	 * @param[in] msg New contact state data
	 */ 
	void contactStateCallback(const gazebo_msgs::ContactsState::ConstPtr& msg, const int toe_idx);

	/**
	 * @brief Publishes current contact state data
	 */ 
	void publishContactState();

	/// Subscriber for toe 0
	ros::Subscriber toe0_contact_state_sub;

	/// Subscriber for toe 1
	ros::Subscriber toe1_contact_state_sub;

	/// Subscriber for toe 2
	ros::Subscriber toe2_contact_state_sub;

	/// Subscriber for toe 3
	ros::Subscriber toe3_contact_state_sub;

	/// ROS publisher for desired GRF
	ros::Publisher grf_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data;
	double update_rate_;

	/// Number of feet
	const int num_feet_ = 4;

	/// Most recent local plan
	spirit_msgs::GRFArray grf_array_msg_;

};


#endif // CONTACT_STATE_PUBLISHER_H
