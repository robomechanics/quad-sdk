#ifndef CONTACT_STATE_PUBLISHER_H
#define CONTACT_STATE_PUBLISHER_H


#include <quad_msgs/GRFArray.h>
#include <quad_utils/ros_utils.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#define MATH_PI 3.141592

//! Publishes contact states from Ignition Gazebo
/*!
   This class subscribes to Ignition Gazebo contact state messages and publishes
   their data under one GRFArray message.
*/

class ContactStatePublisher {
    public:
     /**
      * @brief Constructor for ContactStatePublisher
      * @param[in] nh ROS NodeHandle to publish and subscribe from
      * @return Constructed object of type ContactStatePublisher
      */
     ContactStatePublisher();
     /**
      * @brief Calls ros spinOnce and pubs data at set frequency
      */
     void spin();
   
    private:
     /**
      * @brief Processes new contact state data, GRF data
      * @param[in] msg New contact state data
      */
     void contactStateCallback(const gazebo_msgs::ContactsState::ConstPtr& msg,
                               const int toe_idx);
   
     /**
      * @brief Publishes current contact, force state data
      */
     void publishContactState();
   
    /// Update rate for sending and recieving fata:
    double update_rate_;

    /// Number of feet
    const int num_feet_ = 4;

    /// Most recent local plan
    quad_msgs::msg::GRFArray grf_array_msg_;

    /// Publish ready
    bool ready_to_publish_;
 
};
#endif  // CONTACT_STATE_PUBLISHER_H