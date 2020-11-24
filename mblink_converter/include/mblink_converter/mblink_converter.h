#ifndef MBLINK_CONVERTER_H
#define MBLINK_CONVERTER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommand.h>
#include <spirit_msgs/LegCommandArray.h>
#include <mblink/mblink.hpp>
#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <thread>

using gr::MBLink;

struct LimbCmd_t {
  float pos[3];
  float vel[3];
  float tau[3];
  short kp[3];
  float kd[3];
};

//! Implements online conversion from ROS type to MBLink
/*!
   MBLinkConverter listens for joint control messages and outputs low level commands to the mainboard over mblink
*/
class MBLinkConverter{
public:
  /**
   * @brief Constructor for MBLinkConverter
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @param[in] mblink Pointer to MBLink object
   * @return Constructed object of type EKFEstimator
   */
  MBLinkConverter(ros::NodeHandle nh, int argc, char** argv);

  /**
   * @brief Calls ros spinOnce and spins at set frequency
   */
  void spin();

private:
  /**
   * @brief Callback function to handle new leg command data
   * @param[in] msg spirit_msgs<LegCommandArray> containing pos, vel and torque setpoints and gains
   */
  void legControlCallback(const spirit_msgs::LegCommandArray::ConstPtr& msg);

  /**
   * @brief Compress two floats into one
   * @param[in] in1 First float to be packed
   * @param[out] in2 Second float to be packed
   * @return Compressed float
   */
  float packFloats(float in1, float in2);

  /**
   * @brief Send most recent motor command over mblink
   * @return Boolean signaling successful mblink send
   */
  bool sendMBlink();

  /// Subscriber for motor control messages
  ros::Subscriber leg_control_sub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Last motor control message (keep sending until we get a new message in or node is shutdown)
  spirit_msgs::LegCommandArray::ConstPtr last_leg_command_array_msg_;

  /// Pointer to MBLink object (constructor wants argc and argv, so instantiated in main)
  MBLink mblink_;

};

#endif