#ifndef MBLINK_CONVERTER_H
#define MBLINK_CONVERTER_H

#include <ros/ros.h>
#include <spirit_msgs/MotorCommandArray.h>
#include <mblink/mblink.hpp>
#include <chrono>
#include <thread>

using gr::MBLink;

//! Implements online conversion from ROS type to MBLink
/*!
   MBLinkConverter listens for joint control messages and outputs low level commands to the mainboard over mblink
*/
class MBLinkConverter{
public:
  /**
   * @brief Constructor for MBLinkConverter
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type EKFEstimator
   */
  MBLinkConverter(ros::NodeHandle nh, std::shared_ptr<MBLink> mblink);

  /**
   * @brief Calls ros spinOnce and spins at set frequency
   */
  void spin();

private:
  /**
   * @brief Callback function to handle new motor command data
   * @param[in] msg spirit_msgs<MotorCommandArray> containing pos,vel and torque setpoints and gains
   */
  void motorControlCallback(const spirit_msgs::MotorCommandArray::ConstPtr& msg);

  /**
   * @brief Send most recent motor command over mblink
   */
  bool sendMBlink();

  /// Subscriber for motor control messages
  ros::Subscriber motor_control_sub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Last motor control message (keep sending until we get a new message in or node is shutdown)
  spirit_msgs::MotorCommandArray::ConstPtr last_motor_command_array_msg_;

  /// Pointer to MBLink object
  std::shared_ptr<MBLink> mblink_;

};

#endif