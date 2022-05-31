#ifndef TERRAIN_ESTIMATION_H
#define TERRAIN_ESTIMATION_H

#include <eigen_conversions/eigen_msg.h>
#include <filters/filter_chain.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <quad_msgs/FootPlanDiscrete.h>
#include <quad_msgs/FootState.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/MultiFootPlanContinuous.h>
#include <quad_msgs/MultiFootPlanDiscrete.h>
#include <quad_msgs/MultiFootState.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>
#include <quad_msgs/RobotStateTrajectory.h>
#include <quad_utils/math_utils.h>
#include <quad_utils/quad_kd.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

class TerrainEstimation {
 public:
  /**
   * @brief Constructor for TerrainMapPublisher Class
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type TerrainMapPublisher
   */
  TerrainEstimation(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Callback function to handle new state estimates
   * @param[in] State estimate message contining position and velocity for each
   * joint and robot body
   */
  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  /**
   * @brief Callback function to handle new GRF estimates
   * @param[in] msg the message contining GRF data
   */
  void grfCallback(const quad_msgs::GRFArray::ConstPtr &msg);

  /**
   * @brief Function to publish the footstep history
   */
  void publishFootStepHist();

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// ROS Subscriber for incoming states
  ros::Subscriber robot_state_sub_;

  /// Subscriber for GRF messages
  ros::Subscriber grf_sub_;

  /// GridMap for terrain map data
  grid_map::GridMap terrain_grid_;

  /// Most recent robot state
  quad_msgs::RobotState::ConstPtr robot_state_msg_;

  /// Most recent GRF
  quad_msgs::GRFArray::ConstPtr grf_msg_;

  /// Current state (ground truth or estimate)
  Eigen::VectorXd current_state_;

  // Current positions of each foot
  Eigen::VectorXd current_foot_positions_world_;

  /// Toe radius
  double toe_radius = 0.02;

  /// Footstep history
  grid_map::GridMap terrain_estimation_;

  /// Footstep history publisher
  ros::Publisher terrain_estimation_pub_;

  /// Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  /// Filter chain parameters name.
  std::string filterChainParametersName_;

  /// Temporary index of the foothold history
  std::vector<std::vector<grid_map::Index>> tmp_foot_hist_idx_;

  /// Update rate for sending and receiving data;
  double update_rate_;
};

#endif  // TERRAIN_ESTIMATION_H
