#ifndef GROUND_TRUTH_PUBLISHER_H
#define GROUND_TRUTH_PUBLISHER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <spirit_msgs/StateEstimate.h>
#include <spirit_utils/ros_utils.h>

//! Class to publish the ground truth state data
/*!
   GroundTruthPublisher collects all ground truth sensor data on available topics and merges them into ground truth topics
*/
class GroundTruthPublisher {
public:
  /**
   * @brief Constructor for GroundTruthPublisher
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type GroundTruthPublisher
   */
  GroundTruthPublisher(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

private:
  /**
   * @brief Callback function to handle new mocap data and estimate the velocity
   * @param[in] msg containing new mocap data
   */
  void mocapCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    /**
   * @brief Callback function to handle new joint encoder data
   * @param[in] joint_encoder_msg sensor_msgs<JointState> containing joint pos,vel,current
   */
  void jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new imu data
   * @param[in] imu_msg sensors_msgs<Imu> containing new imu data
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief execute EKF Update step, return state estimate
   * @return state estimate of custom type StateEstimate
   */
  spirit_msgs::StateEstimate updateStep();

  /// Subscriber for joint encoder messages
  ros::Subscriber joint_encoder_sub_;

  /// Subscriber for imu messages
  ros::Subscriber imu_sub_;

  /// Subscriber for mocap messages
  ros::Subscriber mocap_sub_;

  /// Publisher for ground truth state messages
  ros::Publisher ground_truth_state_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// Last state estimate
  spirit_msgs::StateEstimate last_state_est_;

  /// Most recent IMU callback (should be timestamped!)
  sensor_msgs::Imu::ConstPtr last_imu_msg_;

  /// Most recent encoder callback (should be timestamped!)
  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

  /// Last mocap data
  geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_;
  
  /// Best estimate of velocity from mocap diff
  geometry_msgs::Vector3 mocap_vel_estimate_;

  /// Velocity update weight on exponential decay filter
  double alpha_;

};

#endif // GROUND_TRUTH_PUBLISHER_H
