#ifndef SPINE_CONTROLLER_H
#define SPINE_CONTROLLER_H

#include <math.h>
#include <nmpc_controller/nmpc_controller.h>
#include <quad_msgs/LegCommandArray.h>
#include <quad_utils/ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <algorithm>

class SpineController {
 public:
  /**
   * @brief Constructor for SpineController
   * @param[in] nh ROS NodeHandle to publish and subscribe from
   * @return Constructed object of type SpineController
   */
  SpineController(ros::NodeHandle nh);

  /**
   * @brief Calls ros spinOnce and pubs data at set frequency
   */
  void spin();

 private:
  /**
   * @brief Verifies and updates new control mode
   * @param[in] msg New control mode
   */
  void controlModeCallback(const std_msgs::UInt8::ConstPtr& msg);

  void robotPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);

  void robotStateCallback(const quad_msgs::RobotState::ConstPtr &msg);

  void localPlanCallback(const quad_msgs::RobotPlan::ConstPtr &msg);

  void planTrajectory();

  void setupTrajectory();

  /**
   * @brief Compute and send
   * @param[in] elapsed_time Time since node began
   */
  void sendJointPositions(double& elapsed_time);

  /// Nodehandle to pub to and sub from
  ros::NodeHandle nh_;

  /// Publisher for desired joint position
  ros::Publisher joint_control_pub_;

  ros::Subscriber control_mode_sub_;

  ros::Subscriber body_plan_sub_;

  ros::Subscriber robot_state_sub_;

  ros::Subscriber local_plan_sub_;

  quad_msgs::RobotPlan::ConstPtr body_plan_msg_;

  quad_msgs::RobotState::ConstPtr robot_state_msg_;

  quad_msgs::RobotPlan::ConstPtr last_local_plan_msg_;

  Eigen::VectorXd current_state_;
  Eigen::VectorXd current_foot_positions_world_;

  /// Update rate for sending and receiving data;
  double update_rate_;

  /// 0 fixed angle, 1 send to lower level PD
  int control_mode_;

  /// Timestep to interpolate points at
  double interp_dt_;

  std::vector<double> waypoint_angs_;

  std::vector<double> waypoint_ts_;

  /// Target points to hit
  std::vector<double> target_pts_;

  /// Vector of timestamps to hit each target_pt at
  std::vector<double> target_times_;

  /// Gait phase info for each leg
  // not used rn, but something that uses leg phase could be useful!
  std::vector<double> leg_phases_;

  /// Numerically differentiate trajectory for velocity command
  bool use_diff_for_velocity_;

  double set_angle_;

  double open_kp_;

  double open_kd_;

};

#endif  // SPINE_CONTROLLER_H
