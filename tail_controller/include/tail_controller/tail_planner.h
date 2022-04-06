#ifndef TAIL_CONTROLLER_H
#define TAIL_CONTROLLER_H

#include <math.h>
#include <nmpc_controller/nmpc_controller.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_msgs/RobotState.h>
#include <quad_utils/ros_utils.h>
#include <quad_utils/enum_type.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <algorithm>

//! Implements open loop controller
/*!
   OpenLoopController implements a simple, easily configurable open loop
   controller. It transmits desired positions, velocities, feedforward
   torques and gains to the low level controller. Trajectories are
   specified via waypoints and waypoint transition durations.
*/
class TailPlanner {
 public:
  /**
   * @brief Constructor for OpenLoopController
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type OpenLoopController
   */
  TailPlanner(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  void robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);

  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  void localPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  void grfCallback(const quad_msgs::GRFArray::ConstPtr &msg);

  void computeTailPlan();

  ros::NodeHandle nh_;

  double update_rate_;

  ros::Publisher tail_plan_pub_;

  ros::Subscriber body_plan_sub_;

  ros::Subscriber robot_state_sub_;

  ros::Subscriber local_plan_sub_;

  ros::Subscriber cmd_vel_sub_;

  ros::Subscriber grf_sub_;

  std::shared_ptr<NMPCController> tail_planner_;

  quad_msgs::RobotPlan::ConstPtr body_plan_msg_;

  quad_msgs::RobotState::ConstPtr robot_state_msg_;

  quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

  quad_msgs::GRFArray::ConstPtr grf_msg_;

  Eigen::VectorXd current_state_;

  Eigen::MatrixXd ref_body_plan_;

  std::vector<std::vector<bool>> contact_schedule_;

  std::vector<std::vector<bool>> adpative_contact_schedule_;

  Eigen::MatrixXd foot_positions_body_;

  int tail_type_;

  Eigen::VectorXd tail_current_state_;

  Eigen::VectorXd current_foot_positions_world_;

  int current_plan_index_;

  Eigen::MatrixXd ref_tail_plan_;

  Eigen::MatrixXd body_plan_;

  Eigen::MatrixXd grf_plan_;

  Eigen::MatrixXd tail_plan_;

  Eigen::MatrixXd tail_torque_plan_;

  Eigen::VectorXd ref_ground_height_;

  int N_;

  double dt_;

  std::vector<double> cmd_vel_;

  /// Scale for twist cmd_val
  double cmd_vel_scale_;

  /// Nominal robot height
  double z_des_;

  /// Time of the most recent cmd_vel data
  ros::Time last_cmd_vel_msg_time_;

  /// Threshold for waiting for twist cmd_vel data
  double last_cmd_vel_msg_time_max_;

  /// Boolean for using twist input instead of a global body plan
  bool use_twist_input_;

  ros::Time entrance_time_;

  std::vector<bool> miss_contact_leg_;

  /// Time duration to the next plan index
  double first_element_duration_;

  /// If the current solving is duplicated in the same index
  bool same_plan_index_;
};

#endif  // TAIL_CONTROLLER_H
