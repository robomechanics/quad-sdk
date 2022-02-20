#ifndef GROUND_TRUTH_PUBLISHER_H
#define GROUND_TRUTH_PUBLISHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <eigen3/Eigen/Eigen>

//! Class to publish the ground truth state data
/*!
   GroundTruthPublisher collects all ground truth sensor data on available
   topics and merges them into ground truth topics
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
   * @param[in] joint_encoder_msg sensor_msgs<JointState> containing joint
   * pos,vel,current
   */
  void jointEncoderCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * @brief Callback function to handle new imu data
   * @param[in] imu_msg sensors_msgs<Imu> containing new imu data
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /**
   * @brief Callback function to handle new vel data
   * @param[in] vel_msg geometry_msgs<TwistStamped> containing new vel data
   */
  void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  /**
   * @brief execute EKF Update step, return state estimate
   * @param[in] new_state_est state estimate of custom type RobotState
   * @return bool for whether the message is fully populated
   */
  bool updateStep(quad_msgs::RobotState& new_state_est);

  inline double getMedian(const std::vector<double>& v) {
    auto v_sort = v;
    auto m = v_sort.begin() + v_sort.size() / 2;
    std::nth_element(v_sort.begin(), m, v_sort.end());

    return v_sort[v_sort.size() / 2];
  }

  /// Subscriber for joint encoder messages
  ros::Subscriber joint_encoder_sub_;

  /// Subscriber for imu messages
  ros::Subscriber imu_sub_;

  /// Subscriber for vel messages
  ros::Subscriber vel_sub_;

  /// Subscriber for mocap messages
  ros::Subscriber mocap_sub_;

  /// Publisher for ground truth state messages
  ros::Publisher ground_truth_state_pub_;

  /// Publisher for ground truth state in body frame messages
  ros::Publisher ground_truth_state_body_frame_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Update rate of the motion capture system
  double mocap_rate_;

  /// Update rate of the imu
  double imu_rate_;

  /// Update rate of the ground truth publisher node
  double update_rate_;

  /// Last state estimate
  quad_msgs::RobotState last_state_est_;

  /// Most recent IMU callback (should be timestamped!)
  sensor_msgs::Imu::ConstPtr last_imu_msg_;

  /// Most recent vel callback (should be timestamped!)
  geometry_msgs::TwistStamped::ConstPtr last_vel_msg_;

  /// Most recent encoder callback (should be timestamped!)
  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

  /// Last mocap data
  geometry_msgs::PoseStamped::ConstPtr last_mocap_msg_;

  /// Best estimate of velocity from mocap diff
  Eigen::Vector3d imu_vel_estimate_;
  Eigen::Vector3d mocap_vel_estimate_;

  /// Kinematics object
  std::shared_ptr<quad_utils::QuadKD> quadKD_;

  /// Velocity filter time constant
  double filter_time_constant_;

  /// Maximum time elapsed between mocap messages before committing to new
  /// message
  double mocap_dropout_threshold_;

  /// Maximum time elapsed between imu messages before committing to new message
  double imu_dropout_threshold_;

  // Vector containing history of velocity measurements
  std::vector<std::vector<double> > vel_hist_;

  /// Window length for median filter
  const int median_filter_window_ = 3;

  /// Linear velocity filter type
  std::string vel_filter_;

  /// Time of last mocap message
  ros::Time t_mocap_callback_;

  /// RML standard joints order
  std::vector<int> joints_order_;
};

#endif  // GROUND_TRUTH_PUBLISHER_H
