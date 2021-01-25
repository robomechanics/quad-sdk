#ifndef TRAJECTORY_VISUALIZATION_PUBLISHER_H
#define TRAJECTORY_VISUALIZATION_PUBLISHER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <spirit_msgs/StateEstimate.h>
#include <spirit_utils/ros_utils.h>

//! Class to publish the ground truth state data
/*!
   TrajectoryVisualizationPublisher reads a csv file with a trajectory optimization solution, and publishes the ground truth states for visualization
*/
class TrajectoryVisualizationPublisher {
public:
  /**
   * @brief Constructor for TrajectoryVisualizationPublisher
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type TrajectoryVisualizationPublisher
   */
  TrajectoryVisualizationPublisher(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

private:
  /**
   * @brief Loads trajectory data from csv file
  */
  void loadCSV();

  /**
   * @brief Update state to next state in trajectory
   * @return state estimate of custom type StateEstimate
   */
  spirit_msgs::StateEstimate updateStep();

  /// Publisher for ground truth state messages
  ros::Publisher ground_truth_state_pub_;

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Last state estimate
  spirit_msgs::StateEstimate last_state_est_;

  int iterationCount;

};

#endif // TRAJECTORY_VISUALIZATION_PUBLISHER_H
