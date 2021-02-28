#ifndef MBLINK_CONVERTER_H
#define MBLINK_CONVERTER_H

#include <ros/ros.h>
#include <spirit_msgs/LegCommandArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <mblink/mblink.hpp>
#include <Eigen/Dense>
#include <spirit_utils/ros_utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <thread>

using gr::MBLink;

struct LimbCmd_t {
  Eigen::Vector3f pos;
  Eigen::Vector3f vel;
  Eigen::Vector3f tau;
  short kp[3];
  float kd[3];
};

typedef std::unordered_map<std::string, Eigen::VectorXf> RxData_t;

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

  /**
 * @brief Get most recent data payload over mblink and process it
 */
  void publishMBlink();

  /// Subscriber for motor control messages
  ros::Subscriber leg_control_sub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// ROS publisher for joint encoder data
	ros::Publisher joint_encoder_pub_;

  /// ROS publisher for imu data
  ros::Publisher imu_pub_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Last motor control message (keep sending until we get a new message in or node is shutdown)
  spirit_msgs::LegCommandArray::ConstPtr last_leg_command_array_msg_;

  /// Pointer to MBLink object (constructor wants argc and argv, so instantiated in main)
  MBLink mblink_; 

  /// Vector of joint names
  std::vector<std::string> joint_names_ = {"8","0","1","9","2","3","10","4","5","11","6","7"};

  /// Vector denoting joint indices
  std::vector<int> joint_indices_ = {8,0,1,9,2,3,10,4,5,11,6,7};

  /// Vector of kt values for each joint
  std::vector<double> kt_vec_ = {0.546,0.546,1.092,0.546,0.546,1.092,
    0.546,0.546,1.092,0.546,0.546,1.092};

};

#endif